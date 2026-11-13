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

// https://github.com/opencv/opencv/blob/4.x/samples/dnn/text_detection.cpp
// https://github.com/opencv/opencv_zoo/tree/main/models/text_recognition_crnn
/**
 * Text detection and recognition (OCR) with DB (cv::dnn::TextDetectionModel_DB) and CRNN
 * (cv::dnn::TextRecognitionModel), by default DB (ResNet-18, trained on IC15) and CRNN (English) of opencv_zoo
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/detection.hpp>
#include <opencv_apps/msg/detection_array_stamped.hpp>

#include <algorithm>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
namespace
{
// DB (ResNet-18) trained on ICDAR 2015 of opencv_zoo, removed from opencv_zoo after the commit, which OpenCV 4.6 can
// read unlike PP-OCRv3 of text_detection_ppocr, the commits are fixed not to change the files
const char* const DB_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/0d619617a8e9a389150d8c76e417451a19468150/"
                                 "models/text_detection_db/text_detection_DB_IC15_resnet18_2021sep.onnx";
// CRNN of the digits and the lowercase letters, for the gray images of 100x32
const char* const CRNN_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
                                   "models/text_recognition_crnn/text_recognition_CRNN_EN_2021sep.onnx";
const char* const CRNN_VOCABULARY = "0123456789abcdefghijklmnopqrstuvwxyz";
}  // namespace

class DNNTextDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::DetectionArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string detection_model_;
  std::string recognition_model_;
  std::string vocabulary_;
  std::string backend_;
  int input_width_;
  int input_height_;
  double binary_threshold_;
  double polygon_threshold_;
  double unclip_ratio_;
  int max_candidates_;

  std::mutex mutex_;
  std::unique_ptr<cv::dnn::TextDetectionModel_DB> detector_;
  std::unique_ptr<cv::dnn::TextRecognitionModel> recognizer_;  // null when recognition_model is "none"
  bool recognition_gray_;  // the recognition model takes the gray images (e.g. CRNN English), or BGR (CRNN Chinese)
  bool need_model_update_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else
      {
        if (parameter.get_name() == "detection_model")
          detection_model_ = parameter.as_string();
        else if (parameter.get_name() == "recognition_model")
          recognition_model_ = parameter.as_string();
        else if (parameter.get_name() == "vocabulary")
          vocabulary_ = parameter.as_string();
        else if (parameter.get_name() == "backend")
          backend_ = parameter.as_string();
        else if (parameter.get_name() == "input_width")
          input_width_ = parameter.as_int();
        else if (parameter.get_name() == "input_height")
          input_height_ = parameter.as_int();
        else if (parameter.get_name() == "binary_threshold")
          binary_threshold_ = parameter.as_double();
        else if (parameter.get_name() == "polygon_threshold")
          polygon_threshold_ = parameter.as_double();
        else if (parameter.get_name() == "unclip_ratio")
          unclip_ratio_ = parameter.as_double();
        else if (parameter.get_name() == "max_candidates")
          max_candidates_ = parameter.as_int();
        else
          continue;
        need_model_update_ = true;
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // (re)create the models with the parameters, or retry when they failed to load, called with mutex_
  bool updateModel()
  {
    if (!need_model_update_ && detector_)
      return true;
    need_model_update_ = false;
    detector_.reset();
    recognizer_.reset();
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string detection_file =
        modelFile(detection_model_.empty() ? std::string(DB_MODEL_URL) : detection_model_, get_logger());
    if (detection_file.empty())
      return false;
    std::unique_ptr<cv::dnn::TextDetectionModel_DB> detector(new cv::dnn::TextDetectionModel_DB(detection_file));
    detector->setPreferableBackend(static_cast<cv::dnn::Backend>(backend));
    detector->setPreferableTarget(static_cast<cv::dnn::Target>(target));
    detector->setBinaryThreshold(static_cast<float>(binary_threshold_))
        .setPolygonThreshold(static_cast<float>(polygon_threshold_))
        .setUnclipRatio(unclip_ratio_)
        .setMaxCandidates(max_candidates_);
    // the mean of ImageNet in BGR, as the DB models of opencv_zoo
    detector->setInputParams(1.0 / 255, cv::Size(input_width_, input_height_),
                             cv::Scalar(122.67891434, 116.66876762, 104.00698793));
    if (recognition_model_ != "none")
    {
      std::string recognition_file =
          modelFile(recognition_model_.empty() ? std::string(CRNN_MODEL_URL) : recognition_model_, get_logger());
      if (recognition_file.empty())
        return false;
      std::vector<std::string> vocabulary;
      if (vocabulary_.empty())
      {
        for (const char* c = CRNN_VOCABULARY; *c; c++)
          vocabulary.push_back(std::string(1, *c));
      }
      else
      {
        std::string vocabulary_file = modelFile(vocabulary_, get_logger());
        std::ifstream ifs(vocabulary_file);
        std::string line;
        while (std::getline(ifs, line))
        {
          if (!line.empty() && line.back() == '\r')
            line.pop_back();
          vocabulary.push_back(line);
        }
        if (vocabulary.empty())
        {
          RCLCPP_ERROR(get_logger(), "Failed to read the vocabulary %s", vocabulary_.c_str());
          return false;
        }
      }
      // the input channels of the model, 1 (gray) for CRNN English and 3 (BGR) for CRNN Chinese of opencv_zoo,
      // which cannot run on the input of the other channels
      recognition_gray_ = true;
      try
      {
        cv::dnn::Net net = cv::dnn::readNet(recognition_file);
        net.setInput(cv::Mat(std::vector<int>{ 1, 1, 32, 100 }, CV_32F, cv::Scalar(0)));
        net.forward();
      }
      catch (cv::Exception& e)
      {
        recognition_gray_ = false;
      }
      RCLCPP_INFO(get_logger(), "The text recognition model takes %s images", recognition_gray_ ? "gray" : "BGR");
      recognizer_.reset(new cv::dnn::TextRecognitionModel(recognition_file));
      recognizer_->setPreferableBackend(static_cast<cv::dnn::Backend>(backend));
      recognizer_->setPreferableTarget(static_cast<cv::dnn::Target>(target));
      recognizer_->setDecodeType("CTC-greedy");
      recognizer_->setVocabulary(vocabulary);
      recognizer_->setInputParams(1.0 / 127.5, cv::Size(100, 32), cv::Scalar(127.5, 127.5, 127.5));
    }
    detector_ = std::move(detector);
    return true;
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
      opencv_apps::msg::DetectionArrayStamped detections_msg;
      detections_msg.header = msg->header;

      // Do the work
      std::vector<std::vector<cv::Point> > polygons;
      std::vector<float> confidences;
      std::vector<std::string> texts;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s %s",
                                detection_model_.c_str(), recognition_model_.c_str());
          return;
        }
        detector_->detect(frame, polygons, confidences);
        cv::Mat recognition_image = frame;
        if (recognizer_ && recognition_gray_)
          cv::cvtColor(frame, recognition_image, cv::COLOR_BGR2GRAY);
        for (const std::vector<cv::Point>& polygon : polygons)
        {
          std::string text;
          // the corners of the text are bottom-left, top-left, top-right and bottom-right, as the samples of
          // OpenCV, which are transformed to the input of CRNN
          if (recognizer_ && polygon.size() == 4)
          {
            const cv::Size size(100, 32);
            std::vector<cv::Point2f> source(polygon.begin(), polygon.end());
            std::vector<cv::Point2f> target = { cv::Point2f(0, size.height - 1), cv::Point2f(0, 0),
                                                cv::Point2f(size.width - 1, 0),
                                                cv::Point2f(size.width - 1, size.height - 1) };
            cv::Mat cropped;
            cv::warpPerspective(recognition_image, cropped, cv::getPerspectiveTransform(source, target), size);
            text = recognizer_->recognize(cropped);
          }
          texts.push_back(text);
        }
      }

      for (size_t i = 0; i < polygons.size(); i++)
      {
        const std::vector<cv::Point>& polygon = polygons[i];
        cv::Rect box = cv::boundingRect(polygon);
        opencv_apps::msg::Detection detection_msg;
        detection_msg.label = "text";
        detection_msg.data = texts[i];
        detection_msg.id = -1;
        detection_msg.score = i < confidences.size() ? confidences[i] : 1.0;
        detection_msg.rect.x = box.x + box.width / 2.0;
        detection_msg.rect.y = box.y + box.height / 2.0;
        detection_msg.rect.width = box.width;
        detection_msg.rect.height = box.height;
        for (const cv::Point& point : polygon)
        {
          opencv_apps::msg::Point2D point_msg;
          point_msg.x = point.x;
          point_msg.y = point.y;
          detection_msg.corners.push_back(point_msg);
        }
        detections_msg.detections.push_back(detection_msg);

        cv::polylines(frame, polygon, true, cv::Scalar(0, 255, 0), 2);
        if (!texts[i].empty())
        {
          int baseline;
          cv::Size text_size = cv::getTextSize(texts[i], cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
          cv::Point origin(box.x, std::max(box.y - 4, text_size.height));
          cv::rectangle(frame,
                        cv::Rect(origin.x, origin.y - text_size.height - 2, text_size.width, text_size.height + 4),
                        cv::Scalar(0, 255, 0), cv::FILLED);
          cv::putText(frame, texts[i], origin, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        }
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Text detection", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(detections_msg);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNTextDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNTextDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNTextDetectionNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_text_detection", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    need_model_update_ = true;
    recognition_gray_ = true;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    detection_model_ = declare_parameter(
        "detection_model", std::string(""),
        paramDescriptor("Path or URL of the text detection model of DB (onnx), empty for DB (ResNet-18, IC15) of "
                        "opencv_zoo. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time"));
    recognition_model_ = declare_parameter(
        "recognition_model", std::string(""),
        paramDescriptor("Path or URL of the text recognition model of CRNN (onnx), empty for CRNN (English) of "
                        "opencv_zoo, or none to detect the text only"));
    vocabulary_ = declare_parameter("vocabulary", std::string(""),
                                    paramDescriptor("Path or URL of the vocabulary of the recognition model, one "
                                                    "character per line, or empty for 0-9 and a-z of CRNN (English)"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    input_width_ = declare_parameter(
        "input_width", 736, paramDescriptor("Width of the input of the detection model, a multiple of 32", 32, 4096));
    input_height_ = declare_parameter(
        "input_height", 736, paramDescriptor("Height of the input of the detection model, a multiple of 32", 32, 4096));
    binary_threshold_ = declare_parameter("binary_threshold", 0.3,
                                          paramDescriptor("Threshold of the text probability of the pixels", 0.0, 1.0));
    polygon_threshold_ = declare_parameter("polygon_threshold", 0.5,
                                           paramDescriptor("Threshold of the score of the text regions", 0.0, 1.0));
    unclip_ratio_ =
        declare_parameter("unclip_ratio", 2.0, paramDescriptor("Ratio to expand the detected text regions", 0.0, 10.0));
    max_candidates_ =
        declare_parameter("max_candidates", 200, paramDescriptor("Maximum number of the text regions", 1, 10000));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNTextDetectionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::DetectionArrayStamped>("~/detections", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNTextDetectionNodelet)
