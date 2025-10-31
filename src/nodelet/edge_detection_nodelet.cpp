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

#include <rclcpp/rclcpp.hpp>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace opencv_apps
{
class EdgeDetectionNodelet : public opencv_apps::Nodelet
{
  std::shared_ptr<image_transport::Publisher> img_pub_;
  std::shared_ptr<image_transport::Subscriber> img_sub_;
  std::shared_ptr<image_transport::CameraSubscriber> cam_sub_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  int edge_type_;
  int canny_threshold1_;
  int canny_threshold2_;
  int apertureSize_;
  bool L2gradient_;
  bool apply_blur_pre_;
  bool apply_blur_post_;
  int postBlurSize_;
  double postBlurSigma_;
  bool use_camera_info_;

  std::string window_name_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters)
    {
      if (param.get_name() == "edge_type")
      {
        edge_type_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated edge_type to %d", edge_type_);
      }
      else if (param.get_name() == "canny_threshold1")
      {
        canny_threshold1_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated canny_threshold1 to %d", canny_threshold1_);
      }
      else if (param.get_name() == "canny_threshold2")
      {
        canny_threshold2_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated canny_threshold2 to %d", canny_threshold2_);
      }
      else if (param.get_name() == "apertureSize")
      {
        apertureSize_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated apertureSize to %d", apertureSize_);
      }
      else if (param.get_name() == "L2gradient")
      {
        L2gradient_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Updated L2gradient to %s", L2gradient_ ? "true" : "false");
      }
      else if (param.get_name() == "apply_blur_pre")
      {
        apply_blur_pre_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Updated apply_blur_pre to %s", apply_blur_pre_ ? "true" : "false");
      }
      else if (param.get_name() == "apply_blur_post")
      {
        apply_blur_post_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Updated apply_blur_post to %s", apply_blur_post_ ? "true" : "false");
      }
      else if (param.get_name() == "postBlurSize")
      {
        postBlurSize_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated postBlurSize to %d", postBlurSize_);
      }
      else if (param.get_name() == "postBlurSigma")
      {
        postBlurSigma_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Updated postBlurSigma to %.2f", postBlurSigma_);
      }
      else if (param.get_name() == "debug_view")
      {
        debug_view_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Updated debug_view to %s", debug_view_ ? "true" : "false");
      }
    }

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

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    try
    {
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      cv::Mat src_gray;
      cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_RGB2GRAY);
      }
      else
      {
        src_gray = frame;
      }

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      cv::Mat grad;
      switch (edge_type_)
      {
        case 0:  // Sobel
        {
          cv::Mat grad_x, grad_y;
          cv::Mat abs_grad_x, abs_grad_y;
          int scale = 1;
          int delta = 0;
          int ddepth = CV_16S;

          cv::Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
          cv::convertScaleAbs(grad_x, abs_grad_x);

          cv::Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
          cv::convertScaleAbs(grad_y, abs_grad_y);

          cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
          break;
        }
        case 1:  // Laplace
        {
          cv::Mat dst;
          int kernel_size = 3;
          int scale = 1;
          int delta = 0;
          int ddepth = CV_16S;

          cv::Laplacian(src_gray, dst, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
          convertScaleAbs(dst, grad);
          break;
        }
        case 2:  // Canny
        {
          if (apply_blur_pre_)
          {
            cv::blur(src_gray, src_gray, cv::Size(apertureSize_, apertureSize_));
          }

          cv::Canny(src_gray, grad, canny_threshold1_, canny_threshold2_, 3, L2gradient_);

          if (apply_blur_post_)
          {
            cv::GaussianBlur(grad, grad, cv::Size(postBlurSize_, postBlurSize_), postBlurSigma_, postBlurSigma_);
          }
          break;
        }
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, grad);
        int c = cv::waitKey(1);
      }

      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, grad).toImageMsg();
      img_pub_->publish(*out_img);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(),
                   e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
    {
      cam_sub_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this, "image",
          std::bind(&EdgeDetectionNodelet::imageCallbackWithInfo, this,
                    std::placeholders::_1, std::placeholders::_2),
          "raw"));
    }
    else
    {
      img_sub_ = std::make_shared<image_transport::Subscriber>(
        image_transport::create_subscription(
          this, "image",
          std::bind(&EdgeDetectionNodelet::imageCallback, this, std::placeholders::_1),
          "raw"));
    }
  }

  void unsubscribe()
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.reset();
    cam_sub_.reset();
  }

public:
  EdgeDetectionNodelet(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Nodelet("edge_detection", options)
  {
  }

  void onInit() override
  {
    Nodelet::onInit();

    this->declare_parameter("queue_size", 3);
    this->declare_parameter("debug_view", false);
    this->declare_parameter("use_camera_info", false);
    this->declare_parameter("edge_type", 0);
    this->declare_parameter("canny_threshold1", 100);
    this->declare_parameter("canny_threshold2", 200);
    this->declare_parameter("apertureSize", 3);
    this->declare_parameter("L2gradient", false);
    this->declare_parameter("apply_blur_pre", true);
    this->declare_parameter("apply_blur_post", false);
    this->declare_parameter("postBlurSize", 13);
    this->declare_parameter("postBlurSigma", 3.2);

    this->get_parameter("queue_size", queue_size_);
    this->get_parameter("debug_view", debug_view_);
    this->get_parameter("use_camera_info", use_camera_info_);
    this->get_parameter("edge_type", edge_type_);
    this->get_parameter("canny_threshold1", canny_threshold1_);
    this->get_parameter("canny_threshold2", canny_threshold2_);
    this->get_parameter("apertureSize", apertureSize_);
    this->get_parameter("L2gradient", L2gradient_);
    this->get_parameter("apply_blur_pre", apply_blur_pre_);
    this->get_parameter("apply_blur_post", apply_blur_post_);
    this->get_parameter("postBlurSize", postBlurSize_);
    this->get_parameter("postBlurSigma", postBlurSigma_);

    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0);

    window_name_ = "Edge Detection Demo";

    img_pub_ = advertiseImage("image", 1);

    // Register parameter callback for runtime parameter changes
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&EdgeDetectionNodelet::parameterCallback, this, std::placeholders::_1));

    onInitPostProcess();
  }
};

}  // namespace opencv_apps

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::EdgeDetectionNodelet)

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<opencv_apps::EdgeDetectionNodelet>();
  node->onInit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
