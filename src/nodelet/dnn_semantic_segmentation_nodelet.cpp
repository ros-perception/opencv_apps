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

// https://github.com/opencv/opencv/blob/4.x/samples/dnn/segmentation.cpp
// https://github.com/opencv/opencv_zoo/tree/main/models/human_segmentation_pphumanseg
/**
 * Semantic segmentation, by default of the persons with PP-HumanSeg of opencv_zoo, or with a model whose output is
 * the scores of the classes for each pixel (1 x classes x height x width), e.g. FCN, DeepLab, ENet
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <cfloat>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
namespace
{
// PP-HumanSeg of opencv_zoo, the commit is fixed not to change the file
const char* const PPHUMANSEG_MODEL_URL =
    "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
    "models/human_segmentation_pphumanseg/human_segmentation_pphumanseg_2023mar.onnx";
}  // namespace

class DNNSemanticSegmentationNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Publisher label_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

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
  double scale_;
  double mean_;
  bool swap_rb_;

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
      else if (parameter.get_name() == "scale")
        scale_ = parameter.as_double();
      else if (parameter.get_name() == "mean")
        mean_ = parameter.as_double();
      else if (parameter.get_name() == "swap_rb")
        swap_rb_ = parameter.as_bool();
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
    std::string model_file = modelFile(model_.empty() ? std::string(PPHUMANSEG_MODEL_URL) : model_, get_logger());
    if (model_file.empty())
      return false;
    if (!classes_.empty())
    {
      std::string classes_file = modelFile(classes_, get_logger());
      std::ifstream ifs(classes_file);
      if (classes_file.empty() || !ifs)
        RCLCPP_ERROR(get_logger(), "Failed to read the class names %s, use the class ids", classes_.c_str());
      std::string line;
      while (std::getline(ifs, line))
      {
        if (!line.empty() && line.back() == '\r')
          line.pop_back();
        class_names_.push_back(line);
      }
    }
    else if (model_.empty())
    {
      class_names_ = { "background", "person" };
    }
    net_ = cv::dnn::readNet(model_file);
    net_.setPreferableBackend(backend);
    net_.setPreferableTarget(target);
    return true;
  }

  static cv::Vec3b classColor(int class_id)
  {
    return cv::Vec3b((class_id * 67) % 256, (class_id * 131 + 100) % 256, (class_id * 197 + 50) % 256);
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

      // Do the work
      cv::Mat label(frame.size(), CV_8UC1, cv::Scalar(0));
      std::vector<std::string> class_names;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s", model_.c_str());
          return;
        }
        net_.setInput(cv::dnn::blobFromImage(frame, scale_, cv::Size(input_width_, input_height_),
                                             cv::Scalar::all(mean_), swap_rb_, false));
        cv::Mat output = net_.forward();  // 1 x classes x height x width
        class_names = class_names_;
        if (output.dims != 4 || output.size[0] != 1)
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                                "The output of the model is not the scores of the classes for each pixel");
          return;
        }
        // the class of the highest score for each pixel, of the scores resized to the image
        // ~label is mono8, which cannot have more than 256 classes
        int classes = output.size[1];
        if (classes > 256)
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                                "The model has %d classes, more than 256 classes of the mono8 label", classes);
          return;
        }
        cv::Mat best_score(frame.size(), CV_32F, cv::Scalar(-FLT_MAX));
        for (int c = 0; c < classes; c++)
        {
          cv::Mat score(output.size[2], output.size[3], CV_32F, output.ptr<float>(0, c)), resized;
          cv::resize(score, resized, frame.size(), 0, 0, cv::INTER_LINEAR);
          cv::Mat higher = resized > best_score;
          resized.copyTo(best_score, higher);
          label.setTo(cv::Scalar(c), higher);
        }
      }

      // the classes other than the first one (background) are painted on the image
      cv::Mat colored(frame.size(), CV_8UC3);
      for (int y = 0; y < frame.rows; y++)
      {
        for (int x = 0; x < frame.cols; x++)
        {
          int class_id = label.at<uchar>(y, x);
          colored.at<cv::Vec3b>(y, x) = class_id == 0 ? frame.at<cv::Vec3b>(y, x) : classColor(class_id);
        }
      }
      cv::addWeighted(frame, 0.4, colored, 0.6, 0.0, frame);
      double max_class;
      cv::minMaxLoc(label, nullptr, &max_class);
      for (int class_id = 1, line = 0; class_id <= static_cast<int>(max_class); class_id++)
      {
        if (cv::countNonZero(label == class_id) == 0)
          continue;
        std::string name =
            class_id < static_cast<int>(class_names.size()) ? class_names[class_id] : std::to_string(class_id);
        cv::Vec3b color = classColor(class_id);
        cv::Point origin(10, 25 + 25 * line++);
        cv::rectangle(frame, cv::Rect(origin.x, origin.y - 15, 15, 15), cv::Scalar(color[0], color[1], color[2]),
                      cv::FILLED);
        cv::putText(frame, name, origin + cv::Point(20, 0), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Semantic segmentation", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg());
      label_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, label).toImageMsg());
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNSemanticSegmentationNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNSemanticSegmentationNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNSemanticSegmentationNodelet(const rclcpp::NodeOptions& options)
    : Nodelet("dnn_semantic_segmentation", options)
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
        paramDescriptor("Path or URL of the model whose output is the scores of the classes for each pixel, empty "
                        "for PP-HumanSeg of opencv_zoo. The URL is downloaded to ~/.ros/opencv_apps/models/ at the "
                        "first time"));
    classes_ = declare_parameter("classes", std::string(""),
                                 paramDescriptor("Path or URL of the class names of the model, one name per line, the "
                                                 "first one is the background, or empty for background and person "
                                                 "of PP-HumanSeg"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    input_width_ = declare_parameter("input_width", 192, paramDescriptor("Width of the network input", 1, 4096));
    input_height_ = declare_parameter("input_height", 192, paramDescriptor("Height of the network input", 1, 4096));
    scale_ = declare_parameter("scale", 1.0 / 127.5,
                               paramDescriptor("Multiplier of the pixel values for the network input", 0.0, 1.0));
    mean_ = declare_parameter("mean", 127.5,
                              paramDescriptor("Mean subtracted from the pixel values before the scale", 0.0, 255.0));
    swap_rb_ = declare_parameter("swap_rb", true,
                                 paramDescriptor("Swap the red and blue channels, true for the models trained in RGB"));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNSemanticSegmentationNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    label_pub_ = advertiseImage("~/label", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNSemanticSegmentationNodelet)
