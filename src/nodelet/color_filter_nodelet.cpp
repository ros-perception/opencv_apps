// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab.
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
#include "opencv_apps/nodelet.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace opencv_apps
{
class RGBColorFilter;
class HLSColorFilter;
class HSVColorFilter;
class LabColorFilter;

class ColorFilterNodelet : public opencv_apps::Nodelet
{
  friend class RGBColorFilter;
  friend class HLSColorFilter;

protected:
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  bool use_camera_info_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;

  cv::Scalar lower_color_range_;
  cv::Scalar upper_color_range_;

  std::mutex mutex_;

  virtual void filter(const cv::Mat& input_image, cv::Mat& output_image) = 0;

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

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;

      // Do the work
      cv::Mat out_frame;
      filter(frame, out_frame);

      /// Create window
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      std::string new_window_name;

      if (debug_view_)
      {
        if (window_name_ != new_window_name)
        {
          cv::destroyWindow(window_name_);
          window_name_ = new_window_name;
        }
        cv::imshow(window_name_, out_frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::MONO8, out_frame).toImageMsg();
      img_pub_.publish(out_img);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }
  }
  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &ColorFilterNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &ColorFilterNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  // derived classes declare their own parameters and then call onInitPostProcess() at the end of their constructors
  ColorFilterNodelet(const std::string& node_name, const rclcpp::NodeOptions& options) : Nodelet(node_name, options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);

    if (debug_view_)
    {
      always_subscribe_ = true;
    }

    window_name_ = "ColorFilter Demo";

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));

    img_pub_ = advertiseImage("~/image", 1);
  }
};

class RGBColorFilterNodelet : public ColorFilterNodelet
{
protected:
  int r_min_;
  int r_max_;
  int b_min_;
  int b_max_;
  int g_min_;
  int g_max_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "r_limit_max")
        r_max_ = parameter.as_int();
      else if (parameter.get_name() == "r_limit_min")
        r_min_ = parameter.as_int();
      else if (parameter.get_name() == "g_limit_max")
        g_max_ = parameter.as_int();
      else if (parameter.get_name() == "g_limit_min")
        g_min_ = parameter.as_int();
      else if (parameter.get_name() == "b_limit_max")
        b_max_ = parameter.as_int();
      else if (parameter.get_name() == "b_limit_min")
        b_min_ = parameter.as_int();
    }
    updateCondition();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  virtual void updateCondition()
  {
    if (r_max_ < r_min_)
      std::swap(r_max_, r_min_);
    if (g_max_ < g_min_)
      std::swap(g_max_, g_min_);
    if (b_max_ < b_min_)
      std::swap(b_max_, b_min_);
    lower_color_range_ = cv::Scalar(b_min_, g_min_, r_min_);
    upper_color_range_ = cv::Scalar(b_max_, g_max_, r_max_);
  }

  void filter(const cv::Mat& input_image, cv::Mat& output_image)  // NOLINT(modernize-use-override)
  {
    cv::inRange(input_image, lower_color_range_, upper_color_range_, output_image);
  }

public:
  explicit RGBColorFilterNodelet(const rclcpp::NodeOptions& options) : ColorFilterNodelet("rgb_color_filter", options)
  {
    r_max_ = declare_parameter("r_limit_max", 255, paramDescriptor("The maximum allowed field value Red", 0, 255));
    r_min_ = declare_parameter("r_limit_min", 0, paramDescriptor("The minimum allowed field value Red", 0, 255));
    g_max_ = declare_parameter("g_limit_max", 255, paramDescriptor("The maximum allowed field value Green", 0, 255));
    g_min_ = declare_parameter("g_limit_min", 0, paramDescriptor("The minimum allowed field value Green", 0, 255));
    b_max_ = declare_parameter("b_limit_max", 255, paramDescriptor("The maximum allowed field value Blue", 0, 255));
    b_min_ = declare_parameter("b_limit_min", 0, paramDescriptor("The minimum allowed field value Blue", 0, 255));
    updateCondition();
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&RGBColorFilterNodelet::parametersCallback, this, std::placeholders::_1));

    onInitPostProcess();
  }
};

class HLSColorFilterNodelet : public ColorFilterNodelet
{
protected:
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int l_min_;
  int l_max_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "h_limit_max")
        h_max_ = parameter.as_int();
      else if (parameter.get_name() == "h_limit_min")
        h_min_ = parameter.as_int();
      else if (parameter.get_name() == "s_limit_max")
        s_max_ = parameter.as_int();
      else if (parameter.get_name() == "s_limit_min")
        s_min_ = parameter.as_int();
      else if (parameter.get_name() == "l_limit_max")
        l_max_ = parameter.as_int();
      else if (parameter.get_name() == "l_limit_min")
        l_min_ = parameter.as_int();
    }
    updateCondition();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  virtual void updateCondition()
  {
    if (s_max_ < s_min_)
      std::swap(s_max_, s_min_);
    if (l_max_ < l_min_)
      std::swap(l_max_, l_min_);
    lower_color_range_ = cv::Scalar(h_min_ / 2, l_min_, s_min_, 0);
    upper_color_range_ = cv::Scalar(h_max_ / 2, l_max_, s_max_, 0);
  }

  void filter(const cv::Mat& input_image, cv::Mat& output_image)  // NOLINT(modernize-use-override)
  {
    cv::Mat hls_image;
    cv::cvtColor(input_image, hls_image, cv::COLOR_BGR2HLS);
    if (lower_color_range_[0] < upper_color_range_[0])
    {
      cv::inRange(hls_image, lower_color_range_, upper_color_range_, output_image);
    }
    else
    {
      cv::Scalar lower_color_range_0 = cv::Scalar(0, l_min_, s_min_, 0);
      cv::Scalar upper_color_range_0 = cv::Scalar(h_max_ / 2, l_max_, s_max_, 0);
      cv::Scalar lower_color_range_360 = cv::Scalar(h_min_ / 2, l_min_, s_min_, 0);
      cv::Scalar upper_color_range_360 = cv::Scalar(360 / 2, l_max_, s_max_, 0);
      cv::Mat output_image_0, output_image_360;
      cv::inRange(hls_image, lower_color_range_0, upper_color_range_0, output_image_0);
      cv::inRange(hls_image, lower_color_range_360, upper_color_range_360, output_image_360);
      output_image = output_image_0 | output_image_360;
    }
  }

public:
  explicit HLSColorFilterNodelet(const rclcpp::NodeOptions& options) : ColorFilterNodelet("hls_color_filter", options)
  {
    h_max_ = declare_parameter("h_limit_max", 360, paramDescriptor("The maximum allowed field value Hue", 0, 360));
    h_min_ = declare_parameter("h_limit_min", 0, paramDescriptor("The minimum allowed field value Hue", 0, 360));
    s_max_ =
        declare_parameter("s_limit_max", 256, paramDescriptor("The maximum allowed field value Saturation", 0, 256));
    s_min_ = declare_parameter("s_limit_min", 0, paramDescriptor("The minimum allowed field value Saturation", 0, 256));
    l_max_ =
        declare_parameter("l_limit_max", 256, paramDescriptor("The maximum allowed field value Lightness", 0, 256));
    l_min_ = declare_parameter("l_limit_min", 0, paramDescriptor("The minimum allowed field value Lightness", 0, 256));
    updateCondition();
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&HLSColorFilterNodelet::parametersCallback, this, std::placeholders::_1));

    onInitPostProcess();
  }
};

class HSVColorFilterNodelet : public ColorFilterNodelet
{
protected:
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int v_min_;
  int v_max_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "h_limit_max")
        h_max_ = parameter.as_int();
      else if (parameter.get_name() == "h_limit_min")
        h_min_ = parameter.as_int();
      else if (parameter.get_name() == "s_limit_max")
        s_max_ = parameter.as_int();
      else if (parameter.get_name() == "s_limit_min")
        s_min_ = parameter.as_int();
      else if (parameter.get_name() == "v_limit_max")
        v_max_ = parameter.as_int();
      else if (parameter.get_name() == "v_limit_min")
        v_min_ = parameter.as_int();
    }
    updateCondition();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  virtual void updateCondition()
  {
    if (s_max_ < s_min_)
      std::swap(s_max_, s_min_);
    if (v_max_ < v_min_)
      std::swap(v_max_, v_min_);
    lower_color_range_ = cv::Scalar(h_min_ / 2, s_min_, v_min_, 0);
    upper_color_range_ = cv::Scalar(h_max_ / 2, s_max_, v_max_, 0);
  }

  void filter(const cv::Mat& input_image, cv::Mat& output_image)  // NOLINT(modernize-use-override)
  {
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
    if (lower_color_range_[0] < upper_color_range_[0])
    {
      cv::inRange(hsv_image, lower_color_range_, upper_color_range_, output_image);
    }
    else
    {
      cv::Scalar lower_color_range_0 = cv::Scalar(0, s_min_, v_min_, 0);
      cv::Scalar upper_color_range_0 = cv::Scalar(h_max_ / 2, s_max_, v_max_, 0);
      cv::Scalar lower_color_range_360 = cv::Scalar(h_min_ / 2, s_min_, v_min_, 0);
      cv::Scalar upper_color_range_360 = cv::Scalar(360 / 2, s_max_, v_max_, 0);
      cv::Mat output_image_0, output_image_360;
      cv::inRange(hsv_image, lower_color_range_0, upper_color_range_0, output_image_0);
      cv::inRange(hsv_image, lower_color_range_360, upper_color_range_360, output_image_360);
      output_image = output_image_0 | output_image_360;
    }
  }

public:
  explicit HSVColorFilterNodelet(const rclcpp::NodeOptions& options) : ColorFilterNodelet("hsv_color_filter", options)
  {
    h_max_ = declare_parameter("h_limit_max", 360, paramDescriptor("The maximum allowed field value Hue", 0, 360));
    h_min_ = declare_parameter("h_limit_min", 0, paramDescriptor("The minimum allowed field value Hue", 0, 360));
    s_max_ =
        declare_parameter("s_limit_max", 256, paramDescriptor("The maximum allowed field value Saturation", 0, 256));
    s_min_ = declare_parameter("s_limit_min", 0, paramDescriptor("The minimum allowed field value Saturation", 0, 256));
    v_max_ = declare_parameter("v_limit_max", 256, paramDescriptor("The maximum allowed field value Value", 0, 256));
    v_min_ = declare_parameter("v_limit_min", 0, paramDescriptor("The minimum allowed field value Value", 0, 256));
    updateCondition();
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&HSVColorFilterNodelet::parametersCallback, this, std::placeholders::_1));

    onInitPostProcess();
  }
};

class LabColorFilterNodelet : public ColorFilterNodelet
{
protected:
  int l_min_;
  int l_max_;
  int a_min_;
  int a_max_;
  int b_min_;
  int b_max_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "l_limit_max")
        l_max_ = parameter.as_int();
      else if (parameter.get_name() == "l_limit_min")
        l_min_ = parameter.as_int();
      else if (parameter.get_name() == "a_limit_max")
        a_max_ = parameter.as_int();
      else if (parameter.get_name() == "a_limit_min")
        a_min_ = parameter.as_int();
      else if (parameter.get_name() == "b_limit_max")
        b_max_ = parameter.as_int();
      else if (parameter.get_name() == "b_limit_min")
        b_min_ = parameter.as_int();
    }
    updateCondition();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  virtual void updateCondition()
  {
    if (l_max_ < l_min_)
      std::swap(l_max_, l_min_);
    if (a_max_ < a_min_)
      std::swap(a_max_, a_min_);
    if (b_max_ < b_min_)
      std::swap(b_max_, b_min_);
    lower_color_range_ = cv::Scalar(l_min_, a_min_, b_min_);
    upper_color_range_ = cv::Scalar(l_max_, a_max_, b_max_);
  }

  void filter(const cv::Mat& input_image, cv::Mat& output_image)  // NOLINT(modernize-use-override)
  {
    cv::Mat lab_image;
    cv::cvtColor(input_image, lab_image, cv::COLOR_BGR2Lab);
    cv::inRange(lab_image, lower_color_range_, upper_color_range_, output_image);
  }

public:
  explicit LabColorFilterNodelet(const rclcpp::NodeOptions& options) : ColorFilterNodelet("lab_color_filter", options)
  {
    l_max_ = declare_parameter("l_limit_max", 255, paramDescriptor("The maximum allowed field value L*", 0, 255));
    l_min_ = declare_parameter("l_limit_min", 0, paramDescriptor("The minimum allowed field value L*", 0, 255));
    a_max_ = declare_parameter("a_limit_max", 255, paramDescriptor("The maximum allowed field value a*", 0, 255));
    a_min_ = declare_parameter("a_limit_min", 0, paramDescriptor("The minimum allowed field value a*", 0, 255));
    b_max_ = declare_parameter("b_limit_max", 255, paramDescriptor("The maximum allowed field value b*", 0, 255));
    b_min_ = declare_parameter("b_limit_min", 0, paramDescriptor("The minimum allowed field value b*", 0, 255));
    updateCondition();
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&LabColorFilterNodelet::parametersCallback, this, std::placeholders::_1));

    onInitPostProcess();
  }
};

}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::RGBColorFilterNodelet)
RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HLSColorFilterNodelet)
RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HSVColorFilterNodelet)
RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::LabColorFilterNodelet)
