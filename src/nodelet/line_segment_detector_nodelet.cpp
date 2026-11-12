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

// https://github.com/opencv/opencv/blob/3.4/samples/cpp/lsd_lines.cpp
/**
 * @file lsd_lines.cpp
 * @brief Sample code showing how to detect line segments using the LineSegmentDetector
 * @author OpenCV team
 */

#include "opencv_apps/nodelet.h"

#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/line.hpp>
#include <opencv_apps/msg/line_array.hpp>
#include <opencv_apps/msg/line_array_stamped.hpp>

// cv::createLineSegmentDetector() is not available on OpenCV 2.x, and throws on 3.4.6 - 4.5.0 (removed due to
// the license of the original code, restored in 4.5.1), so CMakeLists.txt builds this node only where it works.

namespace opencv_apps
{
class LineSegmentDetectorNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::LineArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;

  std::mutex mutex_;
  cv::Ptr<cv::LineSegmentDetector> lsd_;

  bool use_camera_info_;
  // parameters of cv::createLineSegmentDetector(), the defaults are the ones of OpenCV and cfg/LineSegmentDetector.cfg
  struct LSDParams
  {
    int refine = 0;
    double scale = 0.8;
    double sigma_scale = 0.6;
    double quant = 2.0;
    double angle_threshold = 22.5;
    double log_eps = 0.0;
    double density_threshold = 0.7;
    int n_bins = 1024;
    bool operator==(const LSDParams& o) const
    {
      return refine == o.refine && scale == o.scale && sigma_scale == o.sigma_scale && quant == o.quant &&
             angle_threshold == o.angle_threshold && log_eps == o.log_eps && density_threshold == o.density_threshold &&
             n_bins == o.n_bins;
    }
  };
  LSDParams lsd_params_;
  double lsd_line_length_threshold_;

  // throws cv::Exception for invalid parameters (e.g. scale <= 0)
  static cv::Ptr<cv::LineSegmentDetector> createLSD(const LSDParams& p)
  {
    return cv::createLineSegmentDetector(p.refine, p.scale, p.sigma_scale, p.quant, p.angle_threshold, p.log_eps,
                                         p.density_threshold, p.n_bins);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    bool use_camera_info = use_camera_info_;
    double lsd_line_length_threshold = lsd_line_length_threshold_;
    LSDParams p = lsd_params_;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info = parameter.as_bool();
      else if (parameter.get_name() == "lsd_line_length_threshold")
        lsd_line_length_threshold = parameter.as_double();
      else if (parameter.get_name() == "lsd_refine_type")
        p.refine = parameter.as_int();
      else if (parameter.get_name() == "lsd_scale")
        p.scale = parameter.as_double();
      else if (parameter.get_name() == "lsd_sigma_scale")
        p.sigma_scale = parameter.as_double();
      else if (parameter.get_name() == "lsd_quant")
        p.quant = parameter.as_double();
      else if (parameter.get_name() == "lsd_angle_threshold")
        p.angle_threshold = parameter.as_double();
      else if (parameter.get_name() == "lsd_log_eps")
        p.log_eps = parameter.as_double();
      else if (parameter.get_name() == "lsd_density_threshold")
        p.density_threshold = parameter.as_double();
      else if (parameter.get_name() == "lsd_n_bins")
        p.n_bins = parameter.as_int();
    }
    // cv::LineSegmentDetector has no setters, create it again if its parameters are changed. Apply nothing if they
    // are invalid, so that the previous values (and detector) are kept.
    if (!(p == lsd_params_))
    {
      try
      {
        lsd_ = createLSD(p);
      }
      catch (cv::Exception& e)
      {
        result.successful = false;
        result.reason = "invalid Line Segment Detector parameters: " + e.err;
        RCLCPP_ERROR(get_logger(), "%s", result.reason.c_str());
        return result;
      }
    }
    lsd_params_ = p;
    use_camera_info_ = use_camera_info;
    lsd_line_length_threshold_ = lsd_line_length_threshold;
    result.successful = true;
    return result;
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

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& /*input_frame_from_msg*/)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Work on the image.
    try
    {
      // Convert the image into gray (cv_bridge uses the right conversion for bgr8, rgb8, mono8, ...)
      cv::Mat src_gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;

      // Do the work
      std::vector<cv::Vec4f> lines;
      lsd_->detect(src_gray, lines);

      cv::Mat line_image = cv::Mat::zeros(src_gray.rows, src_gray.cols, CV_8UC1);

      // Messages
      opencv_apps::msg::LineArrayStamped lines_msg;
      lines_msg.header = msg->header;

      for (const cv::Vec4f& l : lines)
      {
        if (std::hypot(l[2] - l[0], l[3] - l[1]) < lsd_line_length_threshold_)
          continue;
        cv::line(line_image, cv::Point(cvRound(l[0]), cvRound(l[1])), cv::Point(cvRound(l[2]), cvRound(l[3])),
                 cv::Scalar(255));

        opencv_apps::msg::Line line_msg;
        line_msg.pt1.x = l[0];
        line_msg.pt1.y = l[1];
        line_msg.pt2.x = l[2];
        line_msg.pt2.y = l[3];
        lines_msg.lines.push_back(line_msg);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::imshow(window_name_, line_image);
        cv::waitKey(1);
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, line_image).toImageMsg());
      // Publish the detected lines.
      msg_pub_->publish(lines_msg);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &LineSegmentDetectorNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &LineSegmentDetectorNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit LineSegmentDetectorNodelet(const rclcpp::NodeOptions& options) : Nodelet("line_segment_detector", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }

    window_name_ = "Line Segment Detector Demo";

    const LSDParams defaults = LSDParams();
    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    lsd_params_.refine = declare_parameter(
        "lsd_refine_type", defaults.refine,
        paramDescriptor("Line Segment Detector Modes, 0: REFINE_NONE, 1: REFINE_STD, 2: REFINE_ADV", 0, 2));
    lsd_params_.scale = declare_parameter(
        "lsd_scale", defaults.scale,
        paramDescriptor("The scale of the image that will be used to find the lines. Range [0.1..1]", 0.1, 1.0));
    lsd_params_.sigma_scale = declare_parameter(
        "lsd_sigma_scale", defaults.sigma_scale,
        paramDescriptor("Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale", 0.1, 10.0));
    lsd_params_.quant =
        declare_parameter("lsd_quant", defaults.quant,
                          paramDescriptor("Bound to the quantization error on the gradient norm", 0.0, 100.0));
    lsd_params_.angle_threshold = declare_parameter("lsd_angle_threshold", defaults.angle_threshold,
                                                    paramDescriptor("Gradient angle tolerance in degrees", 0.1, 179.0));
    lsd_params_.log_eps =
        declare_parameter("lsd_log_eps", defaults.log_eps,
                          paramDescriptor("Detection threshold: -log10(NFA) > log_eps. Used only when "
                                          "advanced refinement is chosen",
                                          0.0, 360.0));
    lsd_params_.density_threshold = declare_parameter(
        "lsd_density_threshold", defaults.density_threshold,
        paramDescriptor("Minimal density of aligned region points in the enclosing rectangle", 0.0, 0.9));
    lsd_params_.n_bins =
        declare_parameter("lsd_n_bins", defaults.n_bins,
                          paramDescriptor("Number of bins in pseudo-ordering of gradient modulus", 1, 10000));
    lsd_line_length_threshold_ = declare_parameter(
        "lsd_line_length_threshold", 100.0,
        paramDescriptor("Threshold of line length. Shorter lines are not published nor drawn.", 0.0, 1000.0));
    try
    {
      lsd_ = createLSD(lsd_params_);
    }
    catch (cv::Exception& e)
    {
      // e.g. _lsd_scale:=0 on ROS1, which does not check the descriptor ranges (on ROS2, declare_parameter() rejects
      // the values out of the ranges at the start, as the other nodes)
      RCLCPP_ERROR(get_logger(), "invalid Line Segment Detector parameters: %s, use the defaults", e.err.c_str());
      lsd_params_ = defaults;
      set_parameters(
          { rclcpp::Parameter("lsd_refine_type", defaults.refine), rclcpp::Parameter("lsd_scale", defaults.scale),
            rclcpp::Parameter("lsd_sigma_scale", defaults.sigma_scale), rclcpp::Parameter("lsd_quant", defaults.quant),
            rclcpp::Parameter("lsd_angle_threshold", defaults.angle_threshold),
            rclcpp::Parameter("lsd_log_eps", defaults.log_eps),
            rclcpp::Parameter("lsd_density_threshold", defaults.density_threshold),
            rclcpp::Parameter("lsd_n_bins", defaults.n_bins) });
      lsd_ = createLSD(lsd_params_);
    }
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&LineSegmentDetectorNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::LineArrayStamped>("~/lines", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::LineSegmentDetectorNodelet)
