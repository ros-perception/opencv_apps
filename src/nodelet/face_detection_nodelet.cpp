/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
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
*   * Neither the name of the Kei Okada nor the names of its
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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/objectDetection/objectDetection.cpp
/**
 * @file objectDetection.cpp
 * @author A. Huaman ( based in the classic facedetect.cpp in samples/c )
 * @brief A simplified version of facedetect.cpp, show how to load a cascade classifier and how to find objects (Face +
 * eyes) in a video stream
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/face.hpp>
#include <opencv_apps/msg/face_array.hpp>
#include <opencv_apps/msg/face_array_stamped.hpp>

namespace opencv_apps
{
class FaceDetectionNodelet : public opencv_apps::Nodelet
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

  cv::CascadeClassifier face_cascade_;
  cv::CascadeClassifier eyes_cascade_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
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
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::msg::FaceArrayStamped faces_msg;
      faces_msg.header = msg->header;

      // Do the work
      std::vector<cv::Rect> faces;
      cv::Mat frame_gray;

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        frame_gray = frame;
      }
      cv::equalizeHist(frame_gray, frame_gray);
//-- Detect faces
#ifndef CV_VERSION_EPOCH
      face_cascade_.detectMultiScale(frame_gray, faces, 1.1, 2, 0, cv::Size(30, 30));
#else
      face_cascade_.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
#endif

      cv::Mat face_image;
      if (!faces.empty())
      {
        cv::Rect max_area = faces[0];
        for (const cv::Rect& face : faces)
        {
          if (max_area.width * max_area.height > face.width * face.height)
          {
            max_area = face;
          }
        }
        face_image = frame(max_area).clone();
      }

      for (const cv::Rect& face : faces)
      {
        cv::Point center(face.x + face.width / 2, face.y + face.height / 2);
        cv::ellipse(frame, center, cv::Size(face.width / 2, face.height / 2), 0, 0, 360, cv::Scalar(255, 0, 255), 2, 8,
                    0);
        opencv_apps::msg::Face face_msg;
        face_msg.face.x = center.x;
        face_msg.face.y = center.y;
        face_msg.face.width = face.width;
        face_msg.face.height = face.height;

        cv::Mat face_roi = frame_gray(face);
        std::vector<cv::Rect> eyes;

//-- In each face, detect eyes
#ifndef CV_VERSION_EPOCH
        eyes_cascade_.detectMultiScale(face_roi, eyes, 1.1, 2, 0, cv::Size(30, 30));
#else
        eyes_cascade_.detectMultiScale(face_roi, eyes, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
#endif

        for (const cv::Rect& eye : eyes)
        {
          cv::Point eye_center(face.x + eye.x + eye.width / 2, face.y + eye.y + eye.height / 2);
          int radius = cvRound((eye.width + eye.height) * 0.25);
          cv::circle(frame, eye_center, radius, cv::Scalar(255, 0, 0), 3, 8, 0);

          opencv_apps::msg::Rect eye_msg;
          eye_msg.x = eye_center.x;
          eye_msg.y = eye_center.y;
          eye_msg.width = eye.width;
          eye_msg.height = eye.height;
          face_msg.eyes.push_back(eye_msg);
        }

        faces_msg.faces.push_back(face_msg);
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
      img_pub_.publish(*out_img);
      msg_pub_->publish(faces_msg);
      if (!faces.empty())
      {
        sensor_msgs::msg::Image::SharedPtr out_face_img =
            cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, face_image).toImageMsg();
        face_img_pub_.publish(*out_face_img);
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
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &FaceDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &FaceDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit FaceDetectionNodelet(const rclcpp::NodeOptions& options) : Nodelet("face_detection", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&FaceDetectionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    face_img_pub_ = advertiseImage("~/face_image", 1);
    msg_pub_ = advertise<opencv_apps::msg::FaceArrayStamped>("~/faces", 1);

    std::string face_cascade_name, eyes_cascade_name;
    face_cascade_name = declare_parameter(
        "face_cascade_name", std::string("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml"));
    eyes_cascade_name = declare_parameter(
        "eyes_cascade_name", std::string("/usr/share/opencv4/haarcascades/haarcascade_eye_tree_eyeglasses.xml"));

    if (!face_cascade_.load(face_cascade_name))
    {
      RCLCPP_ERROR(get_logger(), "--Error loading %s", face_cascade_name.c_str());
    };
    if (!eyes_cascade_.load(eyes_cascade_name))
    {
      RCLCPP_ERROR(get_logger(), "--Error loading %s", eyes_cascade_name.c_str());
    };

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::FaceDetectionNodelet)
