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

// https://github.com/opencv/opencv_zoo/tree/main/models/facial_expression_recognition
/**
 * Facial expression recognition: the faces are detected by YuNet (cv::FaceDetectorYN), aligned by their 5
 * landmarks, and their expressions (angry, disgust, fearful, happy, neutral, sad or surprised) are recognized by
 * MobileFaceNet of opencv_zoo. ~faces has the expression in label and its probability in confidence of each face.
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/dnn.hpp>
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
namespace
{
// MobileFaceNet trained on RAF-DB of opencv_zoo, the commit is fixed not to change the file
const char* const FER_MODEL_URL =
    "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
    "models/facial_expression_recognition/facial_expression_recognition_mobilefacenet_2022july.onnx";
const char* const EXPRESSIONS[] = { "angry", "disgust", "fearful", "happy", "neutral", "sad", "surprised" };
// the 5 landmarks (the right eye, the left eye, the nose tip, the right and left corners of the mouth) of the aligned
// face of 112x112, as opencv_zoo
const float ALIGNED_LANDMARKS[5][2] = { { 38.2946f, 51.6963f },
                                        { 73.5318f, 51.5014f },
                                        { 56.0252f, 71.7366f },
                                        { 41.5493f, 92.3655f },
                                        { 70.7299f, 92.2041f } };
}  // namespace

class DNNFacialExpressionRecognitionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::FaceArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string detection_model_;
  std::string recognition_model_;
  std::string backend_;
  double score_threshold_;
  double nms_threshold_;
  int top_k_;

  std::mutex mutex_;
  cv::Ptr<cv::FaceDetectorYN> detector_;
  cv::dnn::Net recognizer_;
  bool need_model_update_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "detection_model")
        detection_model_ = parameter.as_string();
      else if (parameter.get_name() == "recognition_model")
        recognition_model_ = parameter.as_string();
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
      need_model_update_ = true;
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // (re)create the detector and the recognizer with the parameters, called with mutex_
  void updateModel(const cv::Size& size)
  {
    if (!need_model_update_ && detector_ && !recognizer_.empty())
    {
      if (detector_->getInputSize() != size)
        detector_->setInputSize(size);
      return;
    }
    need_model_update_ = false;
    detector_.release();
    recognizer_ = cv::dnn::Net();
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return;
    std::string detection_file = modelFile(detection_model_.empty() ? YUNET_MODEL_URL : detection_model_, get_logger());
    std::string recognition_file =
        modelFile(recognition_model_.empty() ? std::string(FER_MODEL_URL) : recognition_model_, get_logger());
    if (detection_file.empty() || recognition_file.empty())
      return;
    detector_ = cv::FaceDetectorYN::create(detection_file, "", size, static_cast<float>(score_threshold_),
                                           static_cast<float>(nms_threshold_), top_k_, backend, target);
    recognizer_ = cv::dnn::readNet(recognition_file);
    recognizer_.setPreferableBackend(backend);
    recognizer_.setPreferableTarget(target);
  }

  // the expression of the face (a row of the faces of FaceDetectorYN) and its probability, called with mutex_
  int recognize(const cv::Mat& frame, const cv::Mat& face, float& probability)
  {
    // the face aligned by its 5 landmarks to 112x112, as opencv_zoo
    std::vector<cv::Point2f> landmarks, aligned_landmarks;
    for (int j = 0; j < 5; j++)
    {
      landmarks.push_back(cv::Point2f(face.at<float>(0, 4 + j * 2), face.at<float>(0, 5 + j * 2)));
      aligned_landmarks.push_back(cv::Point2f(ALIGNED_LANDMARKS[j][0], ALIGNED_LANDMARKS[j][1]));
    }
    cv::Mat transform = cv::estimateAffinePartial2D(landmarks, aligned_landmarks, cv::noArray(), cv::LMEDS);
    if (transform.empty())
      return -1;
    cv::Mat aligned;
    cv::warpAffine(frame, aligned, transform, cv::Size(112, 112));
    // in RGB, in -1..1
    recognizer_.setInput(cv::dnn::blobFromImage(aligned, 1.0 / 127.5, cv::Size(), cv::Scalar::all(127.5), true, false));
    cv::Mat scores = recognizer_.forward().reshape(1, 1).clone();  // 1 x expressions
    if (scores.cols != static_cast<int>(sizeof(EXPRESSIONS) / sizeof(EXPRESSIONS[0])))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "The output of the model is not of the 7 expressions");
      return -1;
    }
    // softmax
    double max_score;
    cv::Point max_location;
    cv::minMaxLoc(scores, nullptr, &max_score, nullptr, &max_location);
    cv::exp(scores - max_score, scores);
    probability = static_cast<float>(1.0 / cv::sum(scores)[0]);
    return max_location.x;
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
      std::vector<int> expressions;
      std::vector<float> probabilities;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        updateModel(frame.size());
        if (!detector_ || recognizer_.empty())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Models are not loaded: %s %s",
                                detection_model_.c_str(), recognition_model_.c_str());
          return;
        }
        detector_->detect(frame, faces);
        for (int i = 0; i < faces.rows; i++)
        {
          float probability = 0;
          expressions.push_back(recognize(frame, faces.row(i), probability));
          probabilities.push_back(probability);
        }
      }

      for (int i = 0; i < faces.rows; i++)
      {
        // in the image, as the faces at the border may be partly out of the image
        cv::Rect face = cv::Rect(cvRound(faces.at<float>(i, 0)), cvRound(faces.at<float>(i, 1)),
                                 cvRound(faces.at<float>(i, 2)), cvRound(faces.at<float>(i, 3))) &
                        cv::Rect(0, 0, frame.cols, frame.rows);
        opencv_apps::msg::Face face_msg;
        face_msg.face.x = face.x + face.width / 2.0;
        face_msg.face.y = face.y + face.height / 2.0;
        face_msg.face.width = face.width;
        face_msg.face.height = face.height;
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
        if (expressions[i] >= 0)
        {
          face_msg.label = EXPRESSIONS[expressions[i]];
          face_msg.confidence = probabilities[i];
        }
        faces_msg.faces.push_back(face_msg);

        cv::rectangle(frame, face, cv::Scalar(255, 0, 255), 2);
        cv::putText(frame, cv::format("%s %.2f", face_msg.label.c_str(), face_msg.confidence),
                    cv::Point(face.x, std::max(face.y - 4, 12)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255),
                    1);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Facial expression recognition", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg());
      msg_pub_->publish(faces_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
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
      cam_sub_ =
          subscribeCamera("image", queue_size_, &DNNFacialExpressionRecognitionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNFacialExpressionRecognitionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNFacialExpressionRecognitionNodelet(const rclcpp::NodeOptions& options)
    : Nodelet("dnn_facial_expression_recognition", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    need_model_update_ = true;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    detection_model_ = declare_parameter(
        "detection_model", std::string(""),
        paramDescriptor("Path or URL of the YuNet model (onnx), empty for the default for this OpenCV version. The "
                        "URL is downloaded to ~/.ros/opencv_apps/models/ at the first time"));
    recognition_model_ = declare_parameter(
        "recognition_model", std::string(""),
        paramDescriptor("Path or URL of the facial expression recognition model (onnx), empty for MobileFaceNet of "
                        "opencv_zoo"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    score_threshold_ = declare_parameter("score_threshold", 0.9,
                                         paramDescriptor("Threshold of the score to detect the faces", 0.0, 1.0));
    nms_threshold_ = declare_parameter(
        "nms_threshold", 0.3,
        paramDescriptor("Threshold of the overlap of the faces to suppress, by non maximum suppression", 0.0, 1.0));
    top_k_ = declare_parameter(
        "top_k", 5000, paramDescriptor("Number of the candidates to keep before non maximum suppression", 1, 100000));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNFacialExpressionRecognitionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::FaceArrayStamped>("~/faces", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNFacialExpressionRecognitionNodelet)
