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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/ImgProc/Smoothing.cpp
/**
 * file Smoothing.cpp
 * brief Sample code for simple filters
 * author OpenCV team
 */

#include "opencv_apps/nodelet.h"

#include <iostream>
#include <vector>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

/// Global Variables
int MAX_KERNEL_LENGTH = 31;

namespace opencv_apps
{
enum SmoothingFilterType
{
  Smoothing_Homogeneous_Blur = 0,
  Smoothing_Gaussian_Blur = 1,
  Smoothing_Median_Blur = 2,
  Smoothing_Bilateral_Filter = 3
};

class SmoothingNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  bool use_camera_info_;
  int filter_type_;
  int kernel_size_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "filter_type")
        filter_type_ = parameter.as_int();
      else if (parameter.get_name() == "kernel_size")
        kernel_size_ = (parameter.as_int() / 2) * 2 + 1;  // kernel_size must be odd number
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

  static void trackbarCallback(int /*unused*/, void* /*unused*/)
  {
    need_config_update_ = true;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds
        char kernel_label[] = "Kernel Size : ";

        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::createTrackbar(kernel_label, window_name_, &kernel_size_, MAX_KERNEL_LENGTH, trackbarCallback);
        if (need_config_update_)
        {
          kernel_size_ = (kernel_size_ / 2) * 2 + 1;  // kernel_size must be odd number
          set_parameter(rclcpp::Parameter("kernel_size", kernel_size_));
          need_config_update_ = false;
        }
      }

      cv::Mat out_image = in_image.clone();
      int i = kernel_size_;
      switch (filter_type_)
      {
        case Smoothing_Homogeneous_Blur:
        {
          /// Applying Homogeneous blur
          RCLCPP_DEBUG_STREAM(get_logger(), "Applying Homogeneous blur with kernel size " << i);
          cv::blur(in_image, out_image, cv::Size(i, i), cv::Point(-1, -1));
          break;
        }
        case Smoothing_Gaussian_Blur:
        {
          /// Applying Gaussian blur
          RCLCPP_DEBUG_STREAM(get_logger(), "Applying Gaussian blur with kernel size " << i);
          cv::GaussianBlur(in_image, out_image, cv::Size(i, i), 0, 0);
          break;
        }
        case Smoothing_Median_Blur:
        {
          /// Applying Median blur
          RCLCPP_DEBUG_STREAM(get_logger(), "Applying Median blur with kernel size " << i);
          cv::medianBlur(in_image, out_image, i);
          break;
        }
        case Smoothing_Bilateral_Filter:
        {
          /// Applying Bilateral Filter
          RCLCPP_DEBUG_STREAM(get_logger(), "Applying Bilateral blur with kernel size " << i);
          cv::bilateralFilter(in_image, out_image, i, i * 2, i / 2);
          break;
        }
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, out_image);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      img_pub_.publish(*out_img);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &SmoothingNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &SmoothingNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit SmoothingNodelet(const rclcpp::NodeOptions& options) : Nodelet("smoothing", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "Smoothing Demo";

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    filter_type_ = declare_parameter("filter_type", 1,
                                     paramDescriptor("Smoothing Filter Methods, 0: Homogeneous_Blur, 1: Gaussian_Blur, "
                                                     "2: Median_Blur, 3: Bilateral_Filter",
                                                     0, 3));
    kernel_size_ = declare_parameter(
        "kernel_size", 7,
        paramDescriptor("Size of the kernel (only one because we use a square window). Must be odd.", 1, 31));
    kernel_size_ = (kernel_size_ / 2) * 2 + 1;  // kernel_size must be odd number
    on_set_parameters_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&SmoothingNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);

    onInitPostProcess();
  }
};
bool SmoothingNodelet::need_config_update_ = false;
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::SmoothingNodelet)
