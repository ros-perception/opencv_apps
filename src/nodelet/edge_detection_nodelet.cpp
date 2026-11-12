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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/
/**
 * @file Sobel_Demo.cpp
 * @brief Sample code using Sobel and/orScharr OpenCV functions to make a simple Edge Detector
 * @author OpenCV team
 */
/**
 * @file Laplace_Demo.cpp
 * @brief Sample code showing how to detect edges using the Laplace operator
 * @author OpenCV team
 */
/**
 * @file CannyDetector_Demo.cpp
 * @brief Sample code showing how to detect edges using the Canny Detector
 * @author OpenCV team
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace opencv_apps
{
enum EdgeDetectionType
{
  EdgeDetection_Sobel = 0,
  EdgeDetection_Laplace = 1,
  EdgeDetection_Canny = 2
};

class EdgeDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;
  int edge_type_;
  int canny_threshold1_;
  int canny_threshold2_;
  int apertureSize_;
  bool L2gradient_;
  bool apply_blur_pre_;
  bool apply_blur_post_;
  int postBlurSize_;
  double postBlurSigma_;

  std::string window_name_;
  static bool need_config_update_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "edge_type")
        edge_type_ = parameter.as_int();
      else if (parameter.get_name() == "canny_threshold1")
        canny_threshold1_ = parameter.as_int();
      else if (parameter.get_name() == "canny_threshold2")
        canny_threshold2_ = parameter.as_int();
      else if (parameter.get_name() == "apertureSize")
        apertureSize_ = 2 * ((parameter.as_int() / 2)) + 1;
      else if (parameter.get_name() == "L2gradient")
        L2gradient_ = parameter.as_bool();
      else if (parameter.get_name() == "apply_blur_pre")
        apply_blur_pre_ = parameter.as_bool();
      else if (parameter.get_name() == "apply_blur_post")
        apply_blur_post_ = parameter.as_bool();
      else if (parameter.get_name() == "postBlurSize")
        postBlurSize_ = 2 * ((parameter.as_int()) / 2) + 1;
      else if (parameter.get_name() == "postBlurSigma")
        postBlurSigma_ = parameter.as_double();
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
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Do the work
      cv::Mat src_gray;
      cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

      /// Convert it to gray
      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, src_gray, cv::COLOR_RGB2GRAY);
      }
      else
      {
        src_gray = frame;
      }

      /// Create window
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      std::string new_window_name;
      cv::Mat grad;
      switch (edge_type_)
      {
        case EdgeDetection_Sobel:
        {
          /// Generate grad_x and grad_y
          cv::Mat grad_x, grad_y;
          cv::Mat abs_grad_x, abs_grad_y;

          int scale = 1;
          int delta = 0;
          int ddepth = CV_16S;

          /// Gradient X
          // Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
          cv::Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
          cv::convertScaleAbs(grad_x, abs_grad_x);

          /// Gradient Y
          // Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
          cv::Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
          cv::convertScaleAbs(grad_y, abs_grad_y);

          /// Total Gradient (approximate)
          cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

          new_window_name = "Sobel Edge Detection Demo";
          break;
        }
        case EdgeDetection_Laplace:
        {
          cv::Mat dst;
          int kernel_size = 3;
          int scale = 1;
          int delta = 0;
          int ddepth = CV_16S;
          /// Apply Laplace function

          cv::Laplacian(src_gray, dst, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
          convertScaleAbs(dst, grad);

          new_window_name = "Laplace Edge Detection Demo";
          break;
        }
        case EdgeDetection_Canny:
        {
          int edge_thresh = 1;
          int kernel_size = 3;
          int const max_canny_threshold1 = 500;
          int const max_canny_threshold2 = 500;
          cv::Mat detected_edges;

          /// Reduce noise with a kernel 3x3
          if (apply_blur_pre_)
          {
            cv::blur(src_gray, src_gray, cv::Size(apertureSize_, apertureSize_));
          }

          /// Canny detector
          cv::Canny(src_gray, grad, canny_threshold1_, canny_threshold2_, kernel_size, L2gradient_);
          if (apply_blur_post_)
          {
            cv::GaussianBlur(grad, grad, cv::Size(postBlurSize_, postBlurSize_), postBlurSigma_,
                             postBlurSigma_);  // 0.3*(ksize/2 - 1) + 0.8
          }

          new_window_name = "Canny Edge Detection Demo";

          /// Create a Trackbar for user to enter threshold
          if (debug_view_)
          {
            if (need_config_update_)
            {
              set_parameter(rclcpp::Parameter("canny_threshold1", canny_threshold1_));
              set_parameter(rclcpp::Parameter("canny_threshold2", canny_threshold2_));
              need_config_update_ = false;
            }
            if (window_name_ == new_window_name)
            {
              cv::createTrackbar("Min CannyThreshold1:", window_name_, &canny_threshold1_, max_canny_threshold1,
                                 trackbarCallback);
              cv::createTrackbar("Min CannyThreshold2:", window_name_, &canny_threshold2_, max_canny_threshold2,
                                 trackbarCallback);
            }
          }
          break;
        }
      }

      if (debug_view_)
      {
        if (window_name_ != new_window_name)
        {
          cv::destroyWindow(window_name_);
          window_name_ = new_window_name;
        }
        cv::imshow(window_name_, grad);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, grad).toImageMsg();
      img_pub_.publish(out_img);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &EdgeDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &EdgeDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit EdgeDetectionNodelet(const rclcpp::NodeOptions& options) : Nodelet("edge_detection", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);

    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "Edge Detection Demo";

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    edge_type_ = declare_parameter("edge_type", 0,
                                   paramDescriptor("Edge Detection Methods, 0: Sobel, 1: Laplace, 2: Canny", 0, 2));
    canny_threshold1_ = declare_parameter("canny_threshold1", 100,
                                          paramDescriptor("First threshold for the hysteresis procedure.", 0, 500));
    canny_threshold2_ = declare_parameter("canny_threshold2", 200,
                                          paramDescriptor("Second threshold for the hysteresis procedure.", 0, 500));
    apertureSize_ =
        declare_parameter("apertureSize", 3, paramDescriptor("Aperture size for the Sobel() operator.", 1, 10));
    apertureSize_ = 2 * ((apertureSize_ / 2)) + 1;
    apply_blur_pre_ =
        declare_parameter("apply_blur_pre", true, paramDescriptor("Flag, applying Blur() to input image"));
    postBlurSize_ =
        declare_parameter("postBlurSize", 13, paramDescriptor("Aperture size for the Blur() operator.", 3, 31));
    postBlurSize_ = 2 * ((postBlurSize_) / 2) + 1;
    postBlurSigma_ =
        declare_parameter("postBlurSigma", 3.2, paramDescriptor("Sigma for the GaussianBlur() operator.", 0.0, 10.0));
    apply_blur_post_ = declare_parameter("apply_blur_post", false,
                                         paramDescriptor("Flag, applying GaussianBlur() to output(edge) image"));
    L2gradient_ = declare_parameter(
        "L2gradient", false,
        paramDescriptor("Flag, indicating whether a more accurate  L_2 norm should be used to calculate the image "
                        "gradient magnitude ( L2gradient=true ), or whether the default  L_1 norm is enough "
                        "( L2gradient=false )."));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&EdgeDetectionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);

    onInitPostProcess();
  }
};
bool EdgeDetectionNodelet::need_config_update_ = false;
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::EdgeDetectionNodelet)
