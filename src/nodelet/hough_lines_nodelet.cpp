// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughLines_Demo.cpp
/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/line.hpp>
#include <opencv_apps/msg/line_array.hpp>
#include <opencv_apps/msg/line_array_stamped.hpp>

namespace opencv_apps
{
enum HoughLinesType
{
  HoughLines_Standard_Hough_Transform = 0,
  HoughLines_Probabilistic_Hough_Transform = 1
};

class HoughLinesNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::LineArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  bool use_camera_info_;
  int hough_type_;
  int min_threshold_;
  int max_threshold_;
  int threshold_;
  double rho_;
  double theta_;
  double minLineLength_;
  double maxLineGap_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "hough_type")
        hough_type_ = parameter.as_int();
      else if (parameter.get_name() == "rho")
        rho_ = parameter.as_double();
      else if (parameter.get_name() == "theta")
        theta_ = parameter.as_double();
      else if (parameter.get_name() == "threshold")
        threshold_ = parameter.as_int();
      else if (parameter.get_name() == "minLineLength")
        minLineLength_ = parameter.as_double();
      else if (parameter.get_name() == "maxLineGap")
        maxLineGap_ = parameter.as_double();
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
      cv::Mat src_gray;

      if (in_image.channels() > 1)
      {
        cv::cvtColor(in_image, src_gray, cv::COLOR_BGR2GRAY);
        /// Apply Canny edge detector
        Canny(src_gray, in_image, 50, 200, 3);
      }
      else
      {
        /// Check whether input gray image is filtered such that canny, sobel ...etc
        bool is_filtered = true;
        for (int y = 0; y < in_image.rows; ++y)
        {
          for (int x = 0; x < in_image.cols; ++x)
          {
            if (!(in_image.at<unsigned char>(y, x) == 0 || in_image.at<unsigned char>(y, x) == 255))
            {
              is_filtered = false;
              break;
            }
            if (!is_filtered)
            {
              break;
            }
          }
        }

        if (!is_filtered)
        {
          Canny(in_image, in_image, 50, 200, 3);
        }
      }

      cv::Mat out_image;
      cv::cvtColor(in_image, out_image, CV_GRAY2BGR);

      // Messages
      opencv_apps::msg::LineArrayStamped lines_msg;
      lines_msg.header = msg->header;

      // Do the work
      std::vector<cv::Rect> faces;

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds
        char thresh_label[50];
        sprintf(thresh_label, "Thres: %d + input", min_threshold_);

        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::createTrackbar(thresh_label, window_name_, &threshold_, max_threshold_, trackbarCallback);
        if (need_config_update_)
        {
          set_parameter(rclcpp::Parameter("threshold", threshold_));
          need_config_update_ = false;
        }
      }

      switch (hough_type_)
      {
        case HoughLines_Standard_Hough_Transform:
        {
          std::vector<cv::Vec2f> s_lines;

          /// 1. Use Standard Hough Transform
          cv::HoughLines(in_image, s_lines, rho_, theta_ * CV_PI / 180, threshold_, minLineLength_, maxLineGap_);

          /// Show the result
          for (const cv::Vec2f& s_line : s_lines)
          {
            float r = s_line[0], t = s_line[1];
            double cos_t = cos(t), sin_t = sin(t);
            double x0 = r * cos_t, y0 = r * sin_t;
            double alpha = 1000;

            cv::Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
            cv::Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));
#ifndef CV_VERSION_EPOCH
            cv::line(out_image, pt1, pt2, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
#else
            cv::line(out_image, pt1, pt2, cv::Scalar(255, 0, 0), 3, CV_AA);
#endif
            opencv_apps::msg::Line line_msg;
            line_msg.pt1.x = pt1.x;
            line_msg.pt1.y = pt1.y;
            line_msg.pt2.x = pt2.x;
            line_msg.pt2.y = pt2.y;
            lines_msg.lines.push_back(line_msg);
          }

          break;
        }
        case HoughLines_Probabilistic_Hough_Transform:
        default:
        {
          std::vector<cv::Vec4i> p_lines;

          /// 2. Use Probabilistic Hough Transform
          cv::HoughLinesP(in_image, p_lines, rho_, theta_ * CV_PI / 180, threshold_, minLineLength_, maxLineGap_);

          /// Show the result
          for (const cv::Vec4i& l : p_lines)
          {
#ifndef CV_VERSION_EPOCH
            cv::line(out_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
#else
            cv::line(out_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, CV_AA);
#endif
            opencv_apps::msg::Line line_msg;
            line_msg.pt1.x = l[0];
            line_msg.pt1.y = l[1];
            line_msg.pt2.x = l[2];
            line_msg.pt2.y = l[3];
            lines_msg.lines.push_back(line_msg);
          }

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
      msg_pub_->publish(lines_msg);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &HoughLinesNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &HoughLinesNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit HoughLinesNodelet(const rclcpp::NodeOptions& options) : Nodelet("hough_lines", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "Hough Lines Demo";
    min_threshold_ = 50;
    max_threshold_ = 150;
    threshold_ = max_threshold_;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    hough_type_ = declare_parameter(
        "hough_type", 0,
        paramDescriptor("Hough Line Methods, 0: Standard_Hough_Transform, 1: Probabilistic_Hough_Transform", 0, 1));
    threshold_ = declare_parameter("threshold", 150, paramDescriptor("Hough Line Threshold", 50, 150));
    rho_ = declare_parameter(
        "rho", 1.0, paramDescriptor("The resolution of the parameter r in pixels. We use 1 pixel.", 1.0, 100.0));
    theta_ = declare_parameter(
        "theta", 1.0,
        paramDescriptor("The resolution of the parameter \\theta in radians. We use 1 degree (CV_PI/180)", 1.0, 90.0));
    minLineLength_ = declare_parameter("minLineLength", 30.0,
                                       paramDescriptor("The minimum number of points that can form a line. Lines with "
                                                       "less than this number of points are disregarded.",
                                                       0.0, 500.0));
    maxLineGap_ = declare_parameter(
        "maxLineGap", 10.0,
        paramDescriptor("The maximum gap between two points to be considered in the same line.", 0.0, 100.0));
    on_set_parameters_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&HoughLinesNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::LineArrayStamped>("~/lines", 1);

    onInitPostProcess();
  }
};
bool HoughLinesNodelet::need_config_update_ = false;
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HoughLinesNodelet)
