// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) JSK, 2016 Lab
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

// https://github.com/opencv/opencv/tree/2.4/samples/cpp/tutorial_code/ImgProc/AddingImages.cpp
/**
 * This is a demo of adding image (linear blending).
 */

#include <opencv2/opencv.hpp>

#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/version.h>

#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

#include "opencv_apps/nodelet.h"

namespace opencv_apps
{
class AddingImagesNodelet : public opencv_apps::Nodelet
{
private:
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                                    sensor_msgs::msg::CameraInfo>
      SyncPolicyWithCameraInfo;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                                          sensor_msgs::msg::CameraInfo>
      ApproxSyncPolicyWithCameraInfo;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      ApproxSyncPolicy;

  ////////////////////////////////////////////////////////
  // Parameters
  ////////////////////////////////////////////////////////
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;

  image_transport::SubscriberFilter sub_image1_, sub_image2_;
  image_transport::Publisher img_pub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicyWithCameraInfo> > sync_with_info_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicyWithCameraInfo> > async_with_info_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy> > async_;
  std::mutex mutex_;

  bool approximate_sync_;
  bool use_camera_info_;
  bool auto_beta_;
  double config_beta_;
  double alpha_;
  double beta_;
  double gamma_;

  void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr& msg1,
                             const sensor_msgs::msg::Image::ConstSharedPtr& msg2,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
  {
    doWork(msg1, msg2, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg1,
                     const sensor_msgs::msg::Image::ConstSharedPtr& msg2)
  {
    doWork(msg1, msg2, msg1->header.frame_id);
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    // image_transport takes rclcpp::QoS since lyrical, message_filters since kilted
#if __has_include(<image_transport/node_interfaces.hpp>)
    sub_image1_.subscribe(*this, "image1", "raw", rclcpp::QoS(3));
    sub_image2_.subscribe(*this, "image2", "raw", rclcpp::QoS(3));
#else
    sub_image1_.subscribe(this, "image1", "raw", rclcpp::QoS(3).get_rmw_qos_profile());
    sub_image2_.subscribe(this, "image2", "raw", rclcpp::QoS(3).get_rmw_qos_profile());
#endif
#if RCLCPP_VERSION_GTE(29, 0, 0)
    sub_camera_info_.subscribe(this, "camera_info", rclcpp::QoS(3));
#else
    sub_camera_info_.subscribe(this, "camera_info", rclcpp::QoS(3).get_rmw_qos_profile());
#endif
    if (use_camera_info_)
    {
      if (approximate_sync_)
      {
        async_with_info_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicyWithCameraInfo> >(
            ApproxSyncPolicyWithCameraInfo(queue_size_));
        async_with_info_->connectInput(sub_image1_, sub_image2_, sub_camera_info_);
        async_with_info_->registerCallback(std::bind(&AddingImagesNodelet::imageCallbackWithInfo, this,
                                                     std::placeholders::_1, std::placeholders::_2,
                                                     std::placeholders::_3));
      }
      else
      {
        sync_with_info_ = std::make_shared<message_filters::Synchronizer<SyncPolicyWithCameraInfo> >(
            SyncPolicyWithCameraInfo(queue_size_));
        sync_with_info_->connectInput(sub_image1_, sub_image2_, sub_camera_info_);
        sync_with_info_->registerCallback(std::bind(&AddingImagesNodelet::imageCallbackWithInfo, this,
                                                    std::placeholders::_1, std::placeholders::_2,
                                                    std::placeholders::_3));
      }
    }
    else
    {
      if (approximate_sync_)
      {
        async_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(ApproxSyncPolicy(queue_size_));
        async_->connectInput(sub_image1_, sub_image2_);
        async_->registerCallback(
            std::bind(&AddingImagesNodelet::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
      else
      {
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(queue_size_));
        sync_->connectInput(sub_image1_, sub_image2_);
        sync_->registerCallback(
            std::bind(&AddingImagesNodelet::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
    }
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    sub_image1_.unsubscribe();
    sub_image2_.unsubscribe();
    sub_camera_info_.unsubscribe();
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "alpha")
        alpha_ = parameter.as_double();
      else if (parameter.get_name() == "auto_beta")
        auto_beta_ = parameter.as_bool();
      else if (parameter.get_name() == "beta")
        config_beta_ = parameter.as_double();
      else if (parameter.get_name() == "gamma")
        gamma_ = parameter.as_double();
    }
    if (auto_beta_)
    {
      beta_ = 1.0 - alpha_;
    }
    else
    {
      beta_ = config_beta_;
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg1,
              const sensor_msgs::msg::Image::ConstSharedPtr& image_msg2, const std::string& input_frame_from_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Work on the image.
    try
    {
      cv::Mat image1 = cv_bridge::toCvCopy(image_msg1, image_msg1->encoding)->image;
      cv::Mat image2 = cv_bridge::toCvCopy(image_msg2, image_msg1->encoding)->image;
      if (cv_bridge::getCvType(image_msg1->encoding) != cv_bridge::getCvType(image_msg2->encoding))
      {
        RCLCPP_ERROR(get_logger(), "Encoding of input images must be same type: %s, %s", image_msg1->encoding.c_str(),
                     image_msg2->encoding.c_str());
        return;
      }

      cv::Mat result_image;
      if (image1.rows != image2.rows || image1.cols != image2.cols)
      {
        int new_rows = std::max(image1.rows, image2.rows);
        int new_cols = std::max(image1.cols, image2.cols);
        // if ( new_rows != image1.rows || new_cols != image1.cols ) {
        cv::Mat image1_tmp = cv::Mat(new_rows, new_cols, image1.type());
        image1.copyTo(image1_tmp(cv::Rect(0, 0, image1.cols, image1.rows)));
        image1 = image1_tmp.clone();  // need clone because of toCvShare??

        // if ( new_rows != image2.rows || new_cols != image2.cols ) {
        cv::Mat image2_tmp = cv::Mat(new_rows, new_cols, image2.type());
        image2.copyTo(image2_tmp(cv::Rect(0, 0, image2.cols, image2.rows)));
        image2 = image2_tmp.clone();
      }
      cv::addWeighted(image1, alpha_, image2, beta_, gamma_, result_image);
      //-- Show what you got
      sensor_msgs::msg::Image::SharedPtr image_msg3 =
          cv_bridge::CvImage(image_msg1->header, image_msg1->encoding, result_image).toImageMsg();
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
#ifdef CV_BRIDGE_CVT_COLOR_FOR_DISPLAY_IS_NOT_SUPPORTED
        cv::imshow(window_name_, result_image);
#else
#ifdef CV_BRIDGE_CVT_COLOR_FOR_DISPLAY_OPTION_IS_NOT_SUPPORTED
        cv::imshow(window_name_,
                   cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg3, image_msg3->encoding))->image);
#else
        cv_bridge::CvtColorForDisplayOptions options;
        if (sensor_msgs::image_encodings::bitDepth(image_msg1->encoding) == 32 ||
            sensor_msgs::image_encodings::bitDepth(image_msg1->encoding) == 64)
        {
          // float or double image
          options.do_dynamic_scaling = true;
        }
        cv::imshow(window_name_, cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg3), "", options)->image);
#endif
#endif
        int c = cv::waitKey(1);
      }
      img_pub_.publish(*image_msg3);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }

    prev_stamp_ = image_msg1->header.stamp;
  }

public:
  explicit AddingImagesNodelet(const rclcpp::NodeOptions& options) : Nodelet("adding_images", options)
  {
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "AddingImages Demo";
    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    alpha_ = declare_parameter("alpha", 0.5, paramDescriptor("weight of the first array elements.", 0.0, 1.0));
    auto_beta_ = declare_parameter(
        "auto_beta", true,
        paramDescriptor("True: Automatically set beta weight as 1 - alpha, False: Use user defined beta weight"));
    config_beta_ = declare_parameter("beta", 0.5, paramDescriptor("weight of the second array elements.", 0.0, 1.0));
    beta_ = auto_beta_ ? 1.0 - alpha_ : config_beta_;
    gamma_ = declare_parameter("gamma", 0.0, paramDescriptor("scalar added to each sum.", 0.0, 255.0));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&AddingImagesNodelet::parametersCallback, this, std::placeholders::_1));

    approximate_sync_ = declare_parameter("approximate_sync", true);
    queue_size_ = declare_parameter("queue_size", 100);
    img_pub_ = advertiseImage("~/image", 1);
    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::AddingImagesNodelet)
