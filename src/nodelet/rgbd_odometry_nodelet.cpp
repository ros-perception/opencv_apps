// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Kei Okada.
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


// https://github.com/opencv/opencv_contrib/tree/4.x/modules/rgbd
/**
 * Visual odometry of an RGB-D camera by the odometry of the rgbd module of opencv_contrib: the motion of the camera
 * between the consecutive images by the photometric error of the image (rgbd), the geometric error of the depth
 * (icp) or both (rgbd_icp), accumulated into the pose of the robot (base_frame, by the TF of the camera on it) in
 * odom_frame, published as nav_msgs/Odometry and the TF odom_frame -> base_frame, e.g. for slam_toolbox and Nav2.
 */

#include "opencv_apps/odometry_nodelet.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd.hpp>

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/version.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>

#include <image_transport/subscriber_filter.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

namespace opencv_apps
{
class RGBDOdometryNodelet : public opencv_apps::OdometryNodelet
{
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
                                                                     sensor_msgs::msg::CameraInfo>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  image_transport::SubscriberFilter image_sub_;
  image_transport::SubscriberFilter depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<Sync> sync_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  std::string method_;
  double min_depth_;
  double max_depth_;
  double max_depth_diff_;

  // the odometry and the state, set by the callbacks, which may run in other threads (e.g. the multithreaded nodelet
  // manager of ROS 1)
  std::mutex mutex_;
  bool need_odometry_update_;
  cv::Ptr<cv::rgbd::Odometry> odometry_;
  cv::Mat camera_matrix_;
  cv::Ptr<cv::rgbd::OdometryFrame> previous_frame_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "method")
        method_ = parameter.as_string();
      else if (parameter.get_name() == "min_depth")
        min_depth_ = parameter.as_double();
      else if (parameter.get_name() == "max_depth")
        max_depth_ = parameter.as_double();
      else if (parameter.get_name() == "max_depth_diff")
        max_depth_diff_ = parameter.as_double();
      else
        continue;
      need_odometry_update_ = true;
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // (re)create the odometry of method for the camera matrix, called with mutex_
  bool updateOdometry(const cv::Mat& camera_matrix)
  {
    if (!need_odometry_update_ && odometry_ && cv::norm(camera_matrix, camera_matrix_, cv::NORM_INF) == 0.0)
      return true;
    need_odometry_update_ = false;
    camera_matrix_ = camera_matrix.clone();
    previous_frame_.release();  // the cache of the frame depends on the odometry
    float min_depth = static_cast<float>(min_depth_), max_depth = static_cast<float>(max_depth_);
    float max_depth_diff = static_cast<float>(max_depth_diff_);
    if (method_ == "icp")
      odometry_ = cv::rgbd::ICPOdometry::create(camera_matrix_, min_depth, max_depth, max_depth_diff);
    else if (method_ == "rgbd_icp")
      odometry_ = cv::rgbd::RgbdICPOdometry::create(camera_matrix_, min_depth, max_depth, max_depth_diff);
    else if (method_ == "rgbd")
      odometry_ = cv::rgbd::RgbdOdometry::create(camera_matrix_, min_depth, max_depth, max_depth_diff);
    else
    {
      RCLCPP_ERROR(get_logger(), "Unknown method %s, use rgbd, icp or rgbd_icp", method_.c_str());
      odometry_.release();
      return false;
    }
    return true;
  }

  // the camera matrix of camera_info, adjusted to the image cropped by roi and binned (0 means 1), as aruco_detection
  static cv::Mat cameraMatrix(const sensor_msgs::msg::CameraInfo& info)
  {
// the fields of sensor_msgs/CameraInfo are lower case on ROS2
#ifdef ROS2
    const double* k = info.k.data();
#else
    const double* k = info.K.data();
#endif
    double binning_x = std::max(1u, static_cast<unsigned int>(info.binning_x));
    double binning_y = std::max(1u, static_cast<unsigned int>(info.binning_y));
    return (cv::Mat_<double>(3, 3) << k[0] / binning_x, 0.0, (k[2] - info.roi.x_offset) / binning_x, 0.0,
            k[4] / binning_y, (k[5] - info.roi.y_offset) / binning_y, 0.0, 0.0, 1.0);
  }

  // the depth image in meters
  static cv::Mat depthInMeters(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
  {
    cv::Mat depth = cv_bridge::toCvShare(depth_msg)->image;
    cv::Mat meters;
    if (depth.type() == CV_16UC1)
      depth.convertTo(meters, CV_32FC1, 0.001);  // millimeters, as openni and realsense
    else if (depth.type() == CV_32FC1)
      meters = depth.clone();
    else
      throw std::runtime_error("depth image of encoding " + depth_msg->encoding + " is not 16UC1 nor 32FC1");
    // the odometry takes no depth as NaN, not 0
    meters.setTo(std::numeric_limits<float>::quiet_NaN(), meters <= 0.0f);
    return meters;
  }

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
      std_msgs::msg::Header header = image_msg->header;
      if (!info_msg->header.frame_id.empty())
        header.frame_id = info_msg->header.frame_id;
      if (!sensorOnBase(header.frame_id))
        return;
      cv::Mat camera_matrix = cameraMatrix(*info_msg);
      if (camera_matrix.at<double>(0, 0) == 0.0 || !updateOdometry(camera_matrix))
        return;
      cv::Mat gray;
      cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
      if (image.channels() == 3)
        cv::cvtColor(image, gray,
                     image_msg->encoding == sensor_msgs::image_encodings::RGB8 ? cv::COLOR_RGB2GRAY :
                                                                                 cv::COLOR_BGR2GRAY);
      else if (image.channels() == 4)
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
      else
        gray = image.clone();
      cv::Mat depth = depthInMeters(depth_msg);
      if (gray.size() != depth.size() || gray.depth() != CV_8U)
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                              "the image (8 bit) and the depth image registered to it must be of the same size");
        return;
      }
      // restart the odometry at the first image, or when the time goes back, e.g. a bag played in a loop
      if (restartOdometry(rclcpp::Time(header.stamp)))
        previous_frame_.release();
      cv::Ptr<cv::rgbd::OdometryFrame> frame = cv::rgbd::OdometryFrame::create(gray, depth);
      odometry_->prepareFrameCache(frame, cv::rgbd::OdometryFrame::CACHE_ALL);
      cv::Matx44d motion = cv::Matx44d::eye();  // of the camera from the previous image to this image
      if (previous_frame_)
      {
        // Rt transforms the points of the previous frame into this frame, the inverse of the motion of the camera
        cv::Mat rt;
        if (odometry_->compute(previous_frame_, frame, rt))
          motion = cv::Matx44d(rt).inv();
        else
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "the odometry failed, as if the camera stopped");
      }
      previous_frame_ = frame;
      publishOdometry(header, motion);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
  }

  template <class Filter>
  void subscribeFilter(Filter& filter, const std::string& topic)
  {
#if RCLCPP_VERSION_GTE(29, 0, 0)
    filter.subscribe(this, topic, subscriberQos(queue_size_));
#else
    filter.subscribe(this, topic, subscriberQos(queue_size_).get_rmw_qos_profile());
#endif
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "subscribe");
    // image_transport takes rclcpp::QoS since lyrical
#if __has_include(<image_transport/node_interfaces.hpp>)
    image_sub_.subscribe(*this, "image", image_transport::TransportHints(*this).getTransport(),
                         subscriberQos(queue_size_));
    depth_sub_.subscribe(*this, "depth", "raw", subscriberQos(queue_size_));
#else
    image_sub_.subscribe(this, "image", image_transport::TransportHints(this).getTransport(),
                         subscriberQos(queue_size_).get_rmw_qos_profile());
    depth_sub_.subscribe(this, "depth", "raw", subscriberQos(queue_size_).get_rmw_qos_profile());
#endif
    subscribeFilter(info_sub_, "camera_info");
    sync_ = std::make_shared<Sync>(SyncPolicy(queue_size_));
    sync_->connectInput(image_sub_, depth_sub_, info_sub_);
    sync_->registerCallback(std::bind(&RGBDOdometryNodelet::callback, this, std::placeholders::_1,
                                      std::placeholders::_2, std::placeholders::_3));
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "unsubscribe");
    sync_.reset();
    image_sub_.unsubscribe();
    depth_sub_.unsubscribe();
    info_sub_.unsubscribe();
  }

public:
  explicit RGBDOdometryNodelet(const rclcpp::NodeOptions& options)
    : OdometryNodelet("rgbd_odometry", options), need_odometry_update_(true)
  {
    queue_size_ = declare_parameter("queue_size", 10);
    method_ = declare_parameter(
        "method", std::string("rgbd_icp"),
        paramDescriptor("Method of the odometry: rgbd (the photometric error of the image, cv::rgbd::RgbdOdometry), "
                        "icp (the geometric error of the depth, cv::rgbd::ICPOdometry) or rgbd_icp (both, "
                        "cv::rgbd::RgbdICPOdometry)"));
    min_depth_ = declare_parameter("min_depth", 0.3, paramDescriptor("Minimum valid depth (m).", 0.0, 100.0));
    max_depth_ = declare_parameter("max_depth", 4.0, paramDescriptor("Maximum valid depth (m).", 0.0, 1000.0));
    max_depth_diff_ = declare_parameter(
        "max_depth_diff", 0.07,
        paramDescriptor("Maximum difference (m) of the depths of the corresponding points of the images.", 0.0, 10.0));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&RGBDOdometryNodelet::parametersCallback, this, std::placeholders::_1));

    initOdometry();

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::RGBDOdometryNodelet)
