// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/core/discrete_fourier_transform/discrete_fourier_transform.cpp
/**
 * This is a demo of discrete_fourier_transform image processing,
 */

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "opencv_apps/nodelet.h"

namespace opencv_apps
{
class DiscreteFourierTransformNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::mutex mutex_;

  std::string window_name_;

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
      cam_sub_ = subscribeCamera("image", queue_size_, &DiscreteFourierTransformNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DiscreteFourierTransformNodelet::imageCallback, this);
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
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      cv::Mat src_image = cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
      if (src_image.channels() > 1)
      {
        cv::cvtColor(src_image, src_image, CV_BGR2GRAY);
      }

      cv::Mat padded;  // expand input image to optimal size
      int m = cv::getOptimalDFTSize(src_image.rows);
      int n = cv::getOptimalDFTSize(src_image.cols);  // on the border add zero values
      cv::copyMakeBorder(src_image, padded, 0, m - src_image.rows, 0, n - src_image.cols, cv::BORDER_CONSTANT,
                         cv::Scalar::all(0));

      cv::Mat planes[] = { cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F) };
      cv::Mat complex_image;
      cv::merge(planes, 2, complex_image);    // Add to the expanded another plane with zeros
      cv::dft(complex_image, complex_image);  // this way the result may fit in the source matrix

      // compute the magnitude and switch to logarithmic scale
      // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
      cv::split(complex_image, planes);                // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
      cv::magnitude(planes[0], planes[1], planes[0]);  // planes[0] = magnitude
      cv::Mat magnitude_image = planes[0];
      magnitude_image += cv::Scalar::all(1);  // switch to logarithmic scale
      cv::log(magnitude_image, magnitude_image);

      // crop the spectrum, if it has an odd number of rows or columns
      magnitude_image = magnitude_image(cv::Rect(0, 0, magnitude_image.cols & -2, magnitude_image.rows & -2));
      // rearrange the quadrants of Fourier imagev  so that the origin is at the image center
      int cx = magnitude_image.cols / 2;
      int cy = magnitude_image.rows / 2;

      cv::Mat q0(magnitude_image, cv::Rect(0, 0, cx, cy));    // Top-Left - Create a ROI per quadrant
      cv::Mat q1(magnitude_image, cv::Rect(cx, 0, cx, cy));   // Top-Right
      cv::Mat q2(magnitude_image, cv::Rect(0, cy, cx, cy));   // Bottom-Left
      cv::Mat q3(magnitude_image, cv::Rect(cx, cy, cx, cy));  // Bottom-Right

      cv::Mat tmp;  // swap quadrants (Top-Left with Bottom-Right)
      q0.copyTo(tmp);
      q3.copyTo(q0);
      tmp.copyTo(q3);

      q1.copyTo(tmp);  // swap quadrant (Top-Right with Bottom-Left)
      q2.copyTo(q1);
      tmp.copyTo(q2);
      cv::normalize(magnitude_image, magnitude_image, 0, 255, cv::NORM_MINMAX);

      cv::Mat result_image = cv::Mat::zeros(magnitude_image.rows, magnitude_image.cols, CV_8UC1);
      for (int i = 0; i < magnitude_image.rows; ++i)
      {
        unsigned char* result_data = result_image.ptr<unsigned char>(i);
        float* magnitude_data = magnitude_image.ptr<float>(i);
        for (int j = 0; j < magnitude_image.cols; ++j)
        {
          *result_data++ = (unsigned char)(*magnitude_data++);
        }
      }

      if (debug_view_)
      {
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

    prev_stamp_ = image_msg->header.stamp;
  }

public:
  explicit DiscreteFourierTransformNodelet(const rclcpp::NodeOptions& options)
    : Nodelet("discrete_fourier_transform", options)
  {
    queue_size_ = declare_parameter("queue_size", 1);
    debug_view_ = declare_parameter("debug_view", false);

    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "Discrete Fourier Transform Demo";

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DiscreteFourierTransformNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DiscreteFourierTransformNodelet)
