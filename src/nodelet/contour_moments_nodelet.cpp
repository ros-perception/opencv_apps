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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ShapeDescriptors/moments_demo.cpp
/**
 * @function moments_demo.cpp
 * @brief Demo code to calculate moments
 * @author OpenCV team
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/moment.hpp>
#include <opencv_apps/msg/moment_array.hpp>
#include <opencv_apps/msg/moment_array_stamped.hpp>

namespace opencv_apps
{
// https://stackoverflow.com/questions/13495207/opencv-c-sorting-contours-by-their-contourarea
// comparison function object
bool compareContourAreas(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2)
{
  double i = fabs(contourArea(cv::Mat(contour1)));
  double j = fabs(contourArea(cv::Mat(contour2)));
  return (i > j);
}

class ContourMomentsNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::MomentArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  int low_threshold_;

  bool use_camera_info_;

  std::string window_name_;
  static bool need_config_update_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "canny_low_threshold")
        low_threshold_ = parameter.as_int();
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
      cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;

      // Messages
      opencv_apps::msg::MomentArrayStamped moments_msg;
      moments_msg.header = msg->header;

      // Do the work
      cv::Mat src_gray;
      /// Convert image to gray and blur it
      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        src_gray = frame;
      }
      cv::blur(src_gray, src_gray, cv::Size(3, 3));

      /// Create window
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      cv::Mat canny_output;
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::RNG rng(12345);

      /// Detect edges using canny
      cv::Canny(src_gray, canny_output, low_threshold_, low_threshold_ * 2, 3);
      /// Find contours
      cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

      /// Draw contours
      cv::Mat drawing;
      /// Draw moment on drawing only when img_pub_ have subscribers
      bool publish_drawing = (img_pub_.getNumSubscribers() > 0);
      if (publish_drawing)
      {
        drawing = cv::Mat::zeros(canny_output.size(), CV_8UC3);
      }

      /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
      RCLCPP_INFO(get_logger(), "\t Info: Area and Contour Length");

      // https://stackoverflow.com/questions/13495207/opencv-c-sorting-contours-by-their-contourarea
      std::sort(contours.begin(), contours.end(), compareContourAreas);

      std::vector<cv::Moments> mu(contours.size());
      std::vector<cv::Point2f> mc(contours.size());
      for (size_t i = 0; i < contours.size(); i++)
      {
        /// Get the moments
        for (size_t i = 0; i < contours.size(); i++)
        {
          mu[i] = moments(contours[i], false);
        }

        ///  Get the mass centers:
        for (size_t i = 0; i < contours.size(); i++)
        {
          mc[i] = cv::Point2f(static_cast<float>(mu[i].m10 / mu[i].m00), static_cast<float>(mu[i].m01 / mu[i].m00));
        }

        if (publish_drawing)
        {
          cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
          cv::drawContours(drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point());
          cv::circle(drawing, mc[i], 4, color, -1, 8, 0);
        }
        RCLCPP_INFO(get_logger(),
                    " * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f - Center (%.2f, %.2f)",
                    (int)i, mu[i].m00, cv::contourArea(contours[i]), cv::arcLength(contours[i], true), mc[i].x,
                    mc[i].y);

        opencv_apps::msg::Moment moment_msg;
        moment_msg.m00 = mu[i].m00;
        moment_msg.m10 = mu[i].m10;
        moment_msg.m01 = mu[i].m01;
        moment_msg.m20 = mu[i].m20;
        moment_msg.m11 = mu[i].m11;
        moment_msg.m02 = mu[i].m02;
        moment_msg.m30 = mu[i].m30;
        moment_msg.m21 = mu[i].m21;
        moment_msg.m12 = mu[i].m12;
        moment_msg.m03 = mu[i].m03;
        moment_msg.mu20 = mu[i].mu20;
        moment_msg.mu11 = mu[i].mu11;
        moment_msg.mu02 = mu[i].mu02;
        moment_msg.mu30 = mu[i].mu30;
        moment_msg.mu21 = mu[i].mu21;
        moment_msg.mu12 = mu[i].mu12;
        moment_msg.mu03 = mu[i].mu03;
        moment_msg.nu20 = mu[i].nu20;
        moment_msg.nu11 = mu[i].nu11;
        moment_msg.nu02 = mu[i].nu02;
        moment_msg.nu30 = mu[i].nu30;
        moment_msg.nu21 = mu[i].nu21;
        moment_msg.nu12 = mu[i].nu12;
        moment_msg.nu03 = mu[i].nu03;
        opencv_apps::msg::Point2D center_msg;
        center_msg.x = mc[i].x;
        center_msg.y = mc[i].y;
        moment_msg.center = center_msg;
        moment_msg.area = cv::contourArea(contours[i]);
        moment_msg.length = cv::arcLength(contours[i], true);
        moments_msg.moments.push_back(moment_msg);
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, drawing);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      if (publish_drawing)
      {
        sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(msg->header, "bgr8", drawing).toImageMsg();
        img_pub_.publish(out_img);
      }
      msg_pub_->publish(moments_msg);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &ContourMomentsNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &ContourMomentsNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit ContourMomentsNodelet(const rclcpp::NodeOptions& options) : Nodelet("contour_moments", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "Contours";
    low_threshold_ = 100;  // only for canny

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    low_threshold_ = declare_parameter("canny_low_threshold", 100, paramDescriptor("Canny Edge low Threshold", 1, 255));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&ContourMomentsNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::MomentArrayStamped>("~/moments", 1);
    onInitPostProcess();
  }
};
bool ContourMomentsNodelet::need_config_update_ = false;
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::ContourMomentsNodelet)
