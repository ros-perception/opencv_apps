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

// https://github.com/opencv/opencv/blob/4.x/samples/dnn/classification.cpp
// https://github.com/opencv/opencv_zoo/tree/main/models/image_classification_ppresnet
/**
 * Image classification, by default with PP-ResNet50 (ImageNet 1000 classes) of opencv_zoo, or with a model of the
 * same input (RGB, normalized by the mean and the standard deviation of ImageNet) and output (1 x classes), e.g.
 * MobileNet of opencv_zoo. ~detections has the top_k classes in the order of the score, whose rect is the image.
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
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

namespace opencv_apps
{
namespace
{
// PP-ResNet50 of opencv_zoo, the commit is fixed not to change the file
const char* const PPRESNET_MODEL_URL =
    "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
    "models/image_classification_ppresnet/image_classification_ppresnet50_2022jan.onnx";
// the names of the 1000 classes of ImageNet (ILSVRC2012), one class per line, of the samples of OpenCV
const char* const IMAGENET_CLASSES_URL =
    "https://raw.githubusercontent.com/opencv/opencv/130546e1d90442c32f711122af31260df1644ee5/"
    "samples/data/dnn/classification_classes_ILSVRC2012.txt";
}  // namespace

class DNNImageClassificationNodelet : public opencv_apps::Nodelet
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

  std::string model_;
  std::string classes_;
  std::string backend_;
  int input_width_;
  int input_height_;
  int top_k_;
  bool softmax_;

  std::mutex mutex_;
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  bool need_model_update_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "input_width")
        input_width_ = parameter.as_int();
      else if (parameter.get_name() == "input_height")
        input_height_ = parameter.as_int();
      else if (parameter.get_name() == "top_k")
        top_k_ = parameter.as_int();
      else if (parameter.get_name() == "softmax")
        softmax_ = parameter.as_bool();
      else
      {
        if (parameter.get_name() == "model")
          model_ = parameter.as_string();
        else if (parameter.get_name() == "classes")
          classes_ = parameter.as_string();
        else if (parameter.get_name() == "backend")
          backend_ = parameter.as_string();
        else
          continue;
        need_model_update_ = true;
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // (re)create the model with the parameters, or retry when it failed to load, called with mutex_
  bool updateModel()
  {
    if (!need_model_update_ && !net_.empty())
      return true;
    need_model_update_ = false;
    net_ = cv::dnn::Net();
    class_names_.clear();
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string model_file = modelFile(model_.empty() ? std::string(PPRESNET_MODEL_URL) : model_, get_logger());
    if (model_file.empty())
      return false;
    std::string classes = classes_.empty() ? std::string(IMAGENET_CLASSES_URL) : classes_;
    std::string classes_file = modelFile(classes, get_logger());
    std::ifstream ifs(classes_file);
    if (classes_file.empty() || !ifs)
      RCLCPP_ERROR(get_logger(), "Failed to read the class names %s, use the class ids", classes.c_str());
    std::string line;
    while (std::getline(ifs, line))
    {
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      class_names_.push_back(line);
    }
    net_ = cv::dnn::readNet(model_file);
    net_.setPreferableBackend(backend);
    net_.setPreferableTarget(target);
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
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s", model_.c_str());
          return;
        }
        // the image in RGB, resized to 8/7 of the input and cropped at the center, normalized by the mean and the
        // standard deviation of ImageNet, as opencv_zoo
        cv::Size resized_size(cvRound(input_width_ * 8.0 / 7), cvRound(input_height_ * 8.0 / 7));
        cv::Mat input;
        cv::resize(frame, input, resized_size);
        input = input(cv::Rect((resized_size.width - input_width_) / 2, (resized_size.height - input_height_) / 2,
                               input_width_, input_height_));
        cv::cvtColor(input, input, cv::COLOR_BGR2RGB);
        input.convertTo(input, CV_32FC3, 1.0 / 255);
        cv::subtract(input, cv::Scalar(0.485, 0.456, 0.406), input);
        cv::divide(input, cv::Scalar(0.229, 0.224, 0.225), input);
        net_.setInput(cv::dnn::blobFromImage(input));
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        // the scores of the classes (1 x classes), among the outputs (e.g. the features of PP-ResNet50)
        cv::Mat scores;
        for (const cv::Mat& output : outputs)
        {
          if (output.dims == 2 && output.rows == 1)
          {
            scores = output.reshape(1, 1).clone();
            break;
          }
        }
        if (scores.empty())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "The model has no output of the scores of classes");
          return;
        }
        // the probabilities of the scores (logits), for the models without softmax in them, e.g. PP-ResNet50
        if (softmax_)
        {
          double max_score;
          cv::minMaxLoc(scores, nullptr, &max_score);
          cv::exp(scores - max_score, scores);
          scores /= cv::sum(scores)[0];
        }
        std::vector<int> indices(scores.cols);
        std::iota(indices.begin(), indices.end(), 0);
        int top_k = std::min(top_k_, scores.cols);
        std::partial_sort(indices.begin(), indices.begin() + top_k, indices.end(),
                          [&](int a, int b) { return scores.at<float>(a) > scores.at<float>(b); });
        for (int i = 0; i < top_k; i++)
        {
          int class_id = indices[i];
          opencv_apps::msg::Detection detection_msg;
          // the first name of the class, e.g. "television" of "television, television system"
          std::string name =
              class_id < static_cast<int>(class_names_.size()) ? class_names_[class_id] : std::to_string(class_id);
          detection_msg.label = name.substr(0, name.find(','));
          detection_msg.data = name;
          detection_msg.id = class_id;
          detection_msg.score = scores.at<float>(class_id);
          detection_msg.rect.x = frame.cols / 2.0;
          detection_msg.rect.y = frame.rows / 2.0;
          detection_msg.rect.width = frame.cols;
          detection_msg.rect.height = frame.rows;
          detections_msg.detections.push_back(detection_msg);
        }
      }

      for (size_t i = 0; i < detections_msg.detections.size(); i++)
      {
        const opencv_apps::msg::Detection& detection = detections_msg.detections[i];
        std::string text = cv::format("%s %.2f", detection.label.c_str(), detection.score);
        cv::Point origin(10, 30 + 30 * static_cast<int>(i));
        cv::putText(frame, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 4);
        cv::putText(frame, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Image classification", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg());
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNImageClassificationNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNImageClassificationNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNImageClassificationNodelet(const rclcpp::NodeOptions& options)
    : Nodelet("dnn_image_classification", options)
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
    model_ = declare_parameter(
        "model", std::string(""),
        paramDescriptor("Path or URL of the model (onnx), empty for PP-ResNet50 of opencv_zoo, e.g. MobileNet of "
                        "opencv_zoo (OpenCV 4.9 or later). The URL is downloaded to ~/.ros/opencv_apps/models/ at the "
                        "first time"));
    classes_ = declare_parameter("classes", std::string(""),
                                 paramDescriptor("Path or URL of the class names of the model, one class per line, "
                                                 "or empty for the 1000 classes of ImageNet"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    input_width_ = declare_parameter("input_width", 224, paramDescriptor("Width of the network input", 1, 4096));
    input_height_ = declare_parameter("input_height", 224, paramDescriptor("Height of the network input", 1, 4096));
    top_k_ = declare_parameter("top_k", 5, paramDescriptor("Number of the classes to publish", 1, 1000));
    softmax_ = declare_parameter(
        "softmax", true,
        paramDescriptor("Apply softmax to the output of the model to get the probabilities, true for the models "
                        "whose output is the logits (e.g. PP-ResNet50 and MobileNet of opencv_zoo), false for the "
                        "models whose output is the probabilities"));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNImageClassificationNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::DetectionArrayStamped>("~/detections", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNImageClassificationNodelet)
