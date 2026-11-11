/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Ivan Tarifa.
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
*   * Neither the name of the Ivan Tarifa nor the names of its
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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/Histograms_Matching/EqualizeHist_Demo.cpp
/**
 * @function EqualizeHist_Demo.cpp
 * @brief Demo code for equalizeHist function
 * @author OpenCV team
 */

// https://github.com/opencv/opencv/blob/3.4/samples/tapi/clahe.cpp
/**
 * @function clahe.cpp
 * @brief Demo TApi code for clahe filter
 * @author OpenCV team
 */

#include "opencv_apps/nodelet.h"

#if CV_MAJOR_VERSION >= 3
#include "opencv2/core/ocl.hpp"
#else  // OpenCV Version 2 does not have UMat, use Mat instead
namespace cv
{
typedef Mat UMat;
}
#endif
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace opencv_apps
{
enum EqualizeHistogramType
{
  EqualizeHistogram_EqualizeHist = 0,
  EqualizeHistogram_Clahe = 1
};

class EqualizeHistogramNodelet : public opencv_apps::Nodelet
{
  std::string window_name_;
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  cv::Ptr<cv::CLAHE> clahe_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
#if CV_MAJOR_VERSION >= 3
  bool use_opencl_;
#endif

  bool use_camera_info_;
  int histogram_equalization_type_;
  cv::Size clahe_tile_size_;
  double clahe_clip_limit_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "histogram_equalization_type")
        histogram_equalization_type_ = parameter.as_int();
      else if (parameter.get_name() == "clahe_clip_limit")
        clahe_clip_limit_ = parameter.as_double();
      else if (parameter.get_name() == "clahe_tile_size_x")
        clahe_tile_size_.width = parameter.as_int();
      else if (parameter.get_name() == "clahe_tile_size_y")
        clahe_tile_size_.height = parameter.as_int();
    }
    rcl_interfaces::msg::SetParametersResult result;
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

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    try
    {
      // Convert the image into something opencv can handle.
      cv::UMat frame;
#if CV_MAJOR_VERSION >= 3
      if (msg->encoding == sensor_msgs::image_encodings::BGR8)
        frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.getUMat(cv::ACCESS_RW);
      else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
        frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image.getUMat(cv::ACCESS_RW);
#else
      if (msg->encoding == sensor_msgs::image_encodings::BGR8)
        frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
        frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
#endif

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      // Do the work
      cv::UMat gray, dst;
      if (frame.channels() > 1)
      {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        frame.copyTo(gray);
      }

      switch (histogram_equalization_type_)
      {
        case EqualizeHistogram_Clahe:
          if (clahe_ == nullptr)
            clahe_ = cv::createCLAHE();
          clahe_->setTilesGridSize(clahe_tile_size_);
          clahe_->setClipLimit(clahe_clip_limit_);
          clahe_->apply(gray, dst);
          break;
        case EqualizeHistogram_EqualizeHist:
          equalizeHist(gray, dst);
          break;
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, dst);
        int c = cv::waitKey(1);
      }

      // Publish the image.
#if CV_MAJOR_VERSION >= 3
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, dst.getMat(cv::ACCESS_READ)).toImageMsg();
#else
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, dst).toImageMsg();
#endif
      out_img->header.frame_id = input_frame_from_msg;
      img_pub_.publish(*out_img);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &EqualizeHistogramNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &EqualizeHistogramNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit EqualizeHistogramNodelet(const rclcpp::NodeOptions& options) : Nodelet("equalize_histogram", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
#if CV_MAJOR_VERSION >= 3
    use_opencl_ = declare_parameter("use_opencl", true);
#endif

    window_name_ = "Equalize Histogram Window (" + std::string(get_fully_qualified_name()) + ")";

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    histogram_equalization_type_ =
        declare_parameter("histogram_equalization_type", 0,
                          paramDescriptor("Histogram Equalization Methods, 0: EqualizeHist, 1: Clahe", 0, 1));
    clahe_clip_limit_ =
        declare_parameter("clahe_clip_limit", 20.0, paramDescriptor("Clip Limit for Clahe filter", 0.0, 255.0));
    clahe_tile_size_.width =
        declare_parameter("clahe_tile_size_x", 32, paramDescriptor("X Tile Size for Clahe filter", 0, 300));
    clahe_tile_size_.height =
        declare_parameter("clahe_tile_size_y", 32, paramDescriptor("Y Tile Size for Clahe filter", 0, 300));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&EqualizeHistogramNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);

#if CV_MAJOR_VERSION >= 3
    cv::ocl::setUseOpenCL(use_opencl_);
#endif

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::EqualizeHistogramNodelet)
