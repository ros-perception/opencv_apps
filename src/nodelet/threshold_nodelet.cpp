// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/ImgProc/Threshold.cpp
/**
 * This is a demo of threshold image processing,
 */

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv_apps/nodelet.h"

namespace opencv_apps
{
class ThresholdNodelet : public opencv_apps::Nodelet
{
  ////////////////////////////////////////////////////////
  // Dynamic Reconfigure
  ////////////////////////////////////////////////////////
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;

  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  std::mutex mutex_;
  bool use_camera_info_;
  int threshold_type_;
  int max_binary_value_;
  int threshold_value_;
  bool apply_otsu_;

  void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &ThresholdNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &ThresholdNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "threshold")
        threshold_value_ = parameter.as_int();
      else if (parameter.get_name() == "threshold_type")
        threshold_type_ = parameter.as_int();
      else if (parameter.get_name() == "max_binary")
        max_binary_value_ = parameter.as_int();
      else if (parameter.get_name() == "apply_otsu")
        apply_otsu_ = parameter.as_bool();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const std::string& input_frame_from_msg)
  {
    try
    {
      cv::Mat src_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::Mat gray_image;
      cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);
      cv::Mat result_image;

      // not threshold_type_ |= CV_THRESH_OTSU, which would stay after apply_otsu is set to false
      int threshold_type = threshold_type_;
      if (apply_otsu_)
      {
        threshold_type |= CV_THRESH_OTSU;
      }
      cv::threshold(gray_image, result_image, threshold_value_, max_binary_value_, threshold_type);
      //-- Show what you got
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::imshow(window_name_, result_image);
        int c = cv::waitKey(1);
      }
      img_pub_.publish(
          cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::MONO8, result_image).toImageMsg());
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }
  }

public:
  explicit ThresholdNodelet(const rclcpp::NodeOptions& options) : Nodelet("threshold", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    apply_otsu_ = declare_parameter("apply_otsu", false, paramDescriptor("Apply otsu threshold"));
    threshold_type_ = declare_parameter(
        "threshold_type", 0,
        paramDescriptor("Threshold Type, 0: Binary, 1: Binary_Inverted, 2: Threshold_Truncated, 3: Threshold_To_Zero, "
                        "4: Threshold_To_Zero_Inverted",
                        0, 4));
    threshold_value_ = declare_parameter("threshold", 150, paramDescriptor("Threshold value", 0, 255));
    max_binary_value_ = declare_parameter("max_binary", 255, paramDescriptor("Max Binary value", 0, 255));
    on_set_parameters_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&ThresholdNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::ThresholdNodelet)
