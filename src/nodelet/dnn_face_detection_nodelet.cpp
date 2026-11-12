// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Kei Okada.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// https://github.com/opencv/opencv/blob/4.x/samples/dnn/face_detect.cpp
/**
 * Face detection with YuNet (cv::FaceDetectorYN), https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <opencv_apps/msg/face.hpp>
#include <opencv_apps/msg/face_array.hpp>
#include <opencv_apps/msg/face_array_stamped.hpp>

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
class DNNFaceDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Publisher face_img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::FaceArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string model_;
  std::string backend_;
  double score_threshold_;
  double nms_threshold_;
  int top_k_;

  std::mutex mutex_;
  cv::Ptr<cv::FaceDetectorYN> detector_;
  bool need_detector_update_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "model")
        model_ = parameter.as_string();
      else if (parameter.get_name() == "backend")
        backend_ = parameter.as_string();
      else if (parameter.get_name() == "score_threshold")
        score_threshold_ = parameter.as_double();
      else if (parameter.get_name() == "nms_threshold")
        nms_threshold_ = parameter.as_double();
      else if (parameter.get_name() == "top_k")
        top_k_ = parameter.as_int();
      else
        continue;
      need_detector_update_ = true;
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // (re)create the detector with the parameters, called with mutex_
  void updateDetector(const cv::Size& size)
  {
    if (!need_detector_update_ && detector_)
    {
      if (detector_->getInputSize() != size)
        detector_->setInputSize(size);
      return;
    }
    need_detector_update_ = false;
    detector_.release();
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return;
    std::string model_file = modelFile(model_.empty() ? YUNET_MODEL_URL : model_, get_logger());
    if (model_file.empty())
      return;
    detector_ = cv::FaceDetectorYN::create(model_file, "", size, static_cast<float>(score_threshold_),
                                           static_cast<float>(nms_threshold_), top_k_, backend, target);
  }

  const std::string& frameWithDefault(const std::string& frame, const std::string& image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      if (frame.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Empty image is given");
        return;
      }

      // Messages
      opencv_apps::msg::FaceArrayStamped faces_msg;
      faces_msg.header = msg->header;

      // Do the work
      // each row is x, y, width, height of the face, x, y of the right eye, left eye, nose tip, right and left corners
      // of the mouth, and the score
      cv::Mat faces;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        updateDetector(frame.size());
        if (!detector_)
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Face detector is not loaded, model: %s",
                                model_.c_str());
          return;
        }
        detector_->detect(frame, faces);
      }

      cv::Mat face_image;
      cv::Rect max_area;
      for (int i = 0; i < faces.rows; i++)
      {
        cv::Rect face = cv::Rect(cvRound(faces.at<float>(i, 0)), cvRound(faces.at<float>(i, 1)),
                                 cvRound(faces.at<float>(i, 2)), cvRound(faces.at<float>(i, 3))) &
                        cv::Rect(0, 0, frame.cols, frame.rows);
        if (face.area() > max_area.area())
          max_area = face;

        opencv_apps::msg::Face face_msg;
        face_msg.face.x = face.x + face.width / 2.0;
        face_msg.face.y = face.y + face.height / 2.0;
        face_msg.face.width = face.width;
        face_msg.face.height = face.height;
        face_msg.confidence = faces.at<float>(i, 14);
        // eyes are the landmarks of the eyes with the size of a quarter of the face
        for (int j = 0; j < 2; j++)
        {
          opencv_apps::msg::Rect eye_msg;
          eye_msg.x = faces.at<float>(i, 4 + j * 2);
          eye_msg.y = faces.at<float>(i, 5 + j * 2);
          eye_msg.width = face.width / 4.0;
          eye_msg.height = face.height / 4.0;
          face_msg.eyes.push_back(eye_msg);
        }
        faces_msg.faces.push_back(face_msg);
      }
      if (max_area.area() > 0)
      {
        face_image = frame(max_area).clone();
      }

      for (int i = 0; i < faces.rows; i++)
      {
        // in the image, as the faces at the border may be partly out of the image
        cv::Rect face = cv::Rect(cvRound(faces.at<float>(i, 0)), cvRound(faces.at<float>(i, 1)),
                                 cvRound(faces.at<float>(i, 2)), cvRound(faces.at<float>(i, 3))) &
                        cv::Rect(0, 0, frame.cols, frame.rows);
        cv::rectangle(frame, face, cv::Scalar(255, 0, 255), 2);
        const cv::Scalar colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0),
                                      cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255) };
        for (int j = 0; j < 5; j++)
        {
          cv::circle(frame, cv::Point(cvRound(faces.at<float>(i, 4 + j * 2)), cvRound(faces.at<float>(i, 5 + j * 2))),
                     2, colors[j], 2);
        }
        cv::putText(frame, cv::format("%.2f", faces.at<float>(i, 14)), cv::Point(face.x, std::max(face.y - 4, 12)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
      }
      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Face detection", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(faces_msg);
      if (!face_image.empty())
      {
        sensor_msgs::msg::Image::SharedPtr out_face_img =
            cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, face_image).toImageMsg();
        face_img_pub_.publish(out_face_img);
      }
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNFaceDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNFaceDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNFaceDetectionNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_face_detection", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    need_detector_update_ = true;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    model_ = declare_parameter("model", std::string(""),
                               paramDescriptor("Path or URL of the YuNet model (onnx), empty for the default for this "
                                               "OpenCV version. The URL is downloaded to ~/.ros/opencv_apps/models/ "
                                               "at the first time"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    score_threshold_ = declare_parameter("score_threshold", 0.9,
                                         paramDescriptor("Threshold of the score to detect the faces", 0.0, 1.0));
    nms_threshold_ = declare_parameter(
        "nms_threshold", 0.3,
        paramDescriptor("Threshold of the overlap of the faces to suppress, by non maximum suppression", 0.0, 1.0));
    top_k_ = declare_parameter(
        "top_k", 5000, paramDescriptor("Number of the candidates to keep before non maximum suppression", 1, 100000));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNFaceDetectionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    face_img_pub_ = advertiseImage("~/face_image", 1);
    msg_pub_ = advertise<opencv_apps::msg::FaceArrayStamped>("~/faces", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNFaceDetectionNodelet)
