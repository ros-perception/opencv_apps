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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughCircle_Demo.cpp
/**
 * @file HoughCircle_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/circle.hpp>
#include <opencv_apps/msg/circle_array.hpp>
#include <opencv_apps/msg/circle_array_stamped.hpp>

namespace opencv_apps
{
class HoughCirclesNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::CircleArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  bool use_camera_info_;

  // initial and max values of the parameters of interests.
  int canny_threshold_initial_value_;
  int accumulator_threshold_initial_value_;
  int max_accumulator_threshold_;
  int max_canny_threshold_;
  double canny_threshold_;
  int canny_threshold_int;  // for trackbar
  double accumulator_threshold_;
  int accumulator_threshold_int;
  int gaussian_blur_size_;
  double gaussian_sigma_x_;
  int gaussian_sigma_x_int;
  double gaussian_sigma_y_;
  int gaussian_sigma_y_int;
  int voting_threshold_;
  double min_distance_between_circles_;
  int min_distance_between_circles_int;
  double dp_;
  int dp_int;
  int min_circle_radius_;
  int max_circle_radius_;

  image_transport::Publisher debug_image_pub_;
  int debug_image_type_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "canny_threshold")
        canny_threshold_ = parameter.as_double();
      else if (parameter.get_name() == "accumulator_threshold")
        accumulator_threshold_ = parameter.as_double();
      else if (parameter.get_name() == "gaussian_blur_size")
        gaussian_blur_size_ = parameter.as_int();
      else if (parameter.get_name() == "gaussian_sigma_x")
        gaussian_sigma_x_ = parameter.as_double();
      else if (parameter.get_name() == "gaussian_sigma_y")
        gaussian_sigma_y_ = parameter.as_double();
      else if (parameter.get_name() == "dp")
        dp_ = parameter.as_double();
      else if (parameter.get_name() == "min_circle_radius")
        min_circle_radius_ = parameter.as_int();
      else if (parameter.get_name() == "max_circle_radius")
        max_circle_radius_ = parameter.as_int();
      else if (parameter.get_name() == "debug_image_type")
        debug_image_type_ = parameter.as_int();
      else if (parameter.get_name() == "min_distance_between_circles")
        min_distance_between_circles_ = parameter.as_double();
    }
    updateTrackbarValues();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void updateTrackbarValues()
  {
    canny_threshold_int = int(canny_threshold_);
    accumulator_threshold_int = int(accumulator_threshold_);
    gaussian_sigma_x_int = int(gaussian_sigma_x_);
    gaussian_sigma_y_int = int(gaussian_sigma_y_);
    min_distance_between_circles_int = int(min_distance_between_circles_);
    dp_int = int(dp_);
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

  static void trackbarCallback(int value, void* userdata)
  {
    need_config_update_ = true;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::msg::CircleArrayStamped circles_msg;
      circles_msg.header = msg->header;

      // Do the work
      std::vector<cv::Rect> faces;
      cv::Mat src_gray, edges;

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        src_gray = frame;
      }

      // create the main window, and attach the trackbars
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        cv::createTrackbar("Canny Threshold", window_name_, &canny_threshold_int, max_canny_threshold_,
                           trackbarCallback);
        cv::createTrackbar("Accumulator Threshold", window_name_, &accumulator_threshold_int,
                           max_accumulator_threshold_, trackbarCallback);
        cv::createTrackbar("Gaussian Blur Size", window_name_, &gaussian_blur_size_, 30, trackbarCallback);
        cv::createTrackbar("Gaussian Sigam X", window_name_, &gaussian_sigma_x_int, 10, trackbarCallback);
        cv::createTrackbar("Gaussian Sigma Y", window_name_, &gaussian_sigma_y_int, 10, trackbarCallback);
        cv::createTrackbar("Min Distance between Circles", window_name_, &min_distance_between_circles_int, 100,
                           trackbarCallback);
        cv::createTrackbar("Dp", window_name_, &dp_int, 100, trackbarCallback);
        cv::createTrackbar("Min Circle Radius", window_name_, &min_circle_radius_, 500, trackbarCallback);
        cv::createTrackbar("Max Circle Radius", window_name_, &max_circle_radius_, 2000, trackbarCallback);

        if (need_config_update_)
        {
          canny_threshold_ = (double)canny_threshold_int;
          accumulator_threshold_ = (double)accumulator_threshold_int;
          gaussian_sigma_x_ = (double)gaussian_sigma_x_int;
          gaussian_sigma_y_ = (double)gaussian_sigma_y_int;
          min_distance_between_circles_ = (double)min_distance_between_circles_int;
          dp_ = (double)dp_int;
          set_parameters({ rclcpp::Parameter("canny_threshold", canny_threshold_),
                           rclcpp::Parameter("accumulator_threshold", accumulator_threshold_),
                           rclcpp::Parameter("gaussian_blur_size", gaussian_blur_size_),
                           rclcpp::Parameter("gaussian_sigma_x", gaussian_sigma_x_),
                           rclcpp::Parameter("gaussian_sigma_y", gaussian_sigma_y_),
                           rclcpp::Parameter("min_distance_between_circles", min_distance_between_circles_),
                           rclcpp::Parameter("dp", dp_), rclcpp::Parameter("min_circle_radius", min_circle_radius_),
                           rclcpp::Parameter("max_circle_radius", max_circle_radius_) });
          need_config_update_ = false;
        }
      }

      // Reduce the noise so we avoid false circle detection
      // gaussian_blur_size_ must be odd number
      if (gaussian_blur_size_ % 2 != 1)
      {
        gaussian_blur_size_ = gaussian_blur_size_ + 1;
      }
      cv::GaussianBlur(src_gray, src_gray, cv::Size(gaussian_blur_size_, gaussian_blur_size_), gaussian_sigma_x_,
                       gaussian_sigma_y_);

      // those paramaters cannot be =0
      // so we must check here
      canny_threshold_ = std::max(canny_threshold_, 1.0);
      accumulator_threshold_ = std::max(accumulator_threshold_, 1.0);

      if (debug_view_)
      {
        // https://github.com/Itseez/opencv/blob/2.4.8/modules/imgproc/src/hough.cpp#L817
        cv::Canny(frame, edges, MAX(canny_threshold_ / 2, 1), canny_threshold_, 3);
      }
      if (min_distance_between_circles_ == 0)
      {  // set inital value
        min_distance_between_circles_ = src_gray.rows / 8;
        set_parameter(rclcpp::Parameter("min_distance_between_circles", min_distance_between_circles_));
      }
      // runs the detection, and update the display
      // will hold the results of the detection
      std::vector<cv::Vec3f> circles;
      // runs the actual detection
      cv::HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, dp_, min_distance_between_circles_, canny_threshold_,
                       accumulator_threshold_, min_circle_radius_, max_circle_radius_);

      cv::Mat out_image;
      if (frame.channels() == 1)
      {
        cv::cvtColor(frame, out_image, cv::COLOR_GRAY2BGR);
      }
      else
      {
        out_image = frame;
      }

      // clone the colour, input image for displaying purposes
      for (const cv::Vec3f& i : circles)
      {
        cv::Point center(cvRound(i[0]), cvRound(i[1]));
        int radius = cvRound(i[2]);
        // circle center
        circle(out_image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        circle(out_image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);

        opencv_apps::msg::Circle circle_msg;
        circle_msg.center.x = center.x;
        circle_msg.center.y = center.y;
        circle_msg.radius = radius;
        circles_msg.circles.push_back(circle_msg);
      }

      // shows the results
      if (debug_view_ || debug_image_pub_.getNumSubscribers() > 0)
      {
        cv::Mat debug_image;
        switch (debug_image_type_)
        {
          case 1:
            debug_image = src_gray;
            break;
          case 2:
            debug_image = edges;
            break;
          default:
            debug_image = frame;
            break;
        }
        if (debug_view_)
        {
          cv::imshow(window_name_, debug_image);
          int c = cv::waitKey(1);
          if (c == 's')
          {
            debug_image_type_ = (++debug_image_type_) % 3;
            set_parameter(rclcpp::Parameter("debug_image_type", debug_image_type_));
          }
        }
        if (debug_image_pub_.getNumSubscribers() > 0)
        {
          sensor_msgs::msg::Image::SharedPtr out_debug_img =
              cv_bridge::CvImage(msg->header, msg->encoding, debug_image).toImageMsg();
          debug_image_pub_.publish(*out_debug_img);
        }
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      img_pub_.publish(*out_img);
      msg_pub_->publish(circles_msg);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &HoughCirclesNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &HoughCirclesNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit HoughCirclesNodelet(const rclcpp::NodeOptions& options) : Nodelet("hough_circles", options)
  {
    debug_image_type_ = 0;
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = debug_view_;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "Hough Circle Detection Demo";
    canny_threshold_initial_value_ = 200;
    accumulator_threshold_initial_value_ = 50;
    max_accumulator_threshold_ = 200;
    max_canny_threshold_ = 255;
    min_distance_between_circles_ = 0;

    // declare and initialize both parameters that are subjects to change
    canny_threshold_ = canny_threshold_initial_value_;
    accumulator_threshold_ = accumulator_threshold_initial_value_;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    canny_threshold_ = declare_parameter("canny_threshold", (double)canny_threshold_initial_value_,
                                         paramDescriptor("Canny threshold", 1.0, 255.0));
    accumulator_threshold_ = declare_parameter("accumulator_threshold", (double)accumulator_threshold_initial_value_,
                                               paramDescriptor("Accumulator threshold", 1.0, 200.0));
    gaussian_blur_size_ = declare_parameter("gaussian_blur_size", 9,
                                            paramDescriptor("the size of gaussian blur (should be odd number)", 1, 30));
    gaussian_sigma_x_ =
        declare_parameter("gaussian_sigma_x", 2.0, paramDescriptor("sigma x of gaussian kernel", 1.0, 10.0));
    gaussian_sigma_y_ =
        declare_parameter("gaussian_sigma_y", 2.0, paramDescriptor("sigma y of gaussian kernel", 1.0, 10.0));
    min_distance_between_circles_ =
        declare_parameter("min_distance_between_circles", min_distance_between_circles_,
                          paramDescriptor("mnimum distance between the centers of the detected circles", 0.0, 1024.0));
    dp_ = declare_parameter("dp", 2.0, paramDescriptor("dp", 0.0, 100.0));
    min_circle_radius_ =
        declare_parameter("min_circle_radius", 0, paramDescriptor("the minimum size of the circle", 0, 500));
    max_circle_radius_ =
        declare_parameter("max_circle_radius", 0, paramDescriptor("the maximum size of the circle", 0, 2000));
    debug_image_type_ =
        declare_parameter("debug_image_type", 0,
                          paramDescriptor("Select image type for debug output, 0: Input, 1: Blur, 2: Canny", 0, 2));
    updateTrackbarValues();
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&HoughCirclesNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::CircleArrayStamped>("~/circles", 1);

    debug_image_pub_ = advertiseImage("~/debug_image", 1);

    onInitPostProcess();
  }
};
bool HoughCirclesNodelet::need_config_update_ = false;
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::HoughCirclesNodelet)
