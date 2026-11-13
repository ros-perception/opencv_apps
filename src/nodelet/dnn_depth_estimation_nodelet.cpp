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

// https://github.com/isl-org/MiDaS
/**
 * Monocular depth estimation, by default with MiDaS v2.1 small, whose output is the relative inverse depth (larger
 * is nearer, without the scale and the shift), or with a model of the same input and output
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
namespace
{
// MiDaS v2.1 small (MIT License) of the release v2_1 of https://github.com/isl-org/MiDaS
const char* const MIDAS_MODEL_URL = "https://github.com/isl-org/MiDaS/releases/download/v2_1/model-small.onnx";
}  // namespace

class DNNDepthEstimationNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string model_;
  std::string backend_;
  int input_width_;
  int input_height_;

  std::mutex mutex_;
  cv::dnn::Net net_;
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
      else
      {
        if (parameter.get_name() == "model")
          model_ = parameter.as_string();
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
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string model_file = modelFile(model_.empty() ? std::string(MIDAS_MODEL_URL) : model_, get_logger());
    if (model_file.empty())
      return false;
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

      // Do the work
      cv::Mat inverse_depth;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s", model_.c_str());
          return;
        }
        // the image in RGB, normalized by the mean and the standard deviation of ImageNet, as MiDaS
        cv::Mat input;
        cv::resize(frame, input, cv::Size(input_width_, input_height_), 0, 0, cv::INTER_CUBIC);
        cv::cvtColor(input, input, cv::COLOR_BGR2RGB);
        input.convertTo(input, CV_32FC3, 1.0 / 255);
        cv::subtract(input, cv::Scalar(0.485, 0.456, 0.406), input);
        cv::divide(input, cv::Scalar(0.229, 0.224, 0.225), input);
        net_.setInput(cv::dnn::blobFromImage(input));
        cv::Mat output = net_.forward();  // 1 x height x width, or 1 x 1 x height x width
        if (output.total() != static_cast<size_t>(output.size[output.dims - 2]) * output.size[output.dims - 1])
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "The output of the model is not a depth image");
          return;
        }
        cv::Mat depth(output.size[output.dims - 2], output.size[output.dims - 1], CV_32F, output.ptr<float>());
        cv::resize(depth, inverse_depth, frame.size(), 0, 0, cv::INTER_CUBIC);
      }

      // the relative inverse depth in color, normalized by the range of the image
      double min_value, max_value;
      cv::minMaxLoc(inverse_depth, &min_value, &max_value);
      cv::Mat normalized;
      inverse_depth.convertTo(normalized, CV_8U, 255.0 / std::max(max_value - min_value, 1e-6),
                              -min_value * 255.0 / std::max(max_value - min_value, 1e-6));
      cv::Mat colored;
      cv::applyColorMap(normalized, colored, cv::COLORMAP_INFERNO);

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Depth estimation", colored);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, colored).toImageMsg());
      depth_pub_.publish(
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, inverse_depth).toImageMsg());
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNDepthEstimationNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNDepthEstimationNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNDepthEstimationNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_depth_estimation", options)
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
        paramDescriptor("Path or URL of the model (onnx) of the same input and output as MiDaS, empty for MiDaS v2.1 "
                        "small. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    input_width_ = declare_parameter("input_width", 256,
                                     paramDescriptor("Width of the network input, 256 for MiDaS v2.1 small", 1, 4096));
    input_height_ = declare_parameter(
        "input_height", 256, paramDescriptor("Height of the network input, 256 for MiDaS v2.1 small", 1, 4096));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNDepthEstimationNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    depth_pub_ = advertiseImage("~/inverse_depth", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNDepthEstimationNodelet)
