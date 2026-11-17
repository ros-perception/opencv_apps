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
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

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
#include <cmath>
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
  // the depth image and camera_info, without the image (use_image false)
  using DepthSyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using DepthSync = message_filters::Synchronizer<DepthSyncPolicy>;

  image_transport::SubscriberFilter image_sub_;
  image_transport::SubscriberFilter depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<Sync> sync_;
  std::shared_ptr<DepthSync> depth_sync_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  std::string method_;
  bool use_image_;
  double min_depth_;
  double max_depth_;
  double max_depth_diff_;

  // the odometry and the state, set by the callbacks, which may run in other threads (e.g. the multithreaded nodelet
  // manager of ROS 1)
  std::mutex mutex_;
  bool need_odometry_update_;
  cv::Ptr<cv::rgbd::Odometry> odometry_;
  cv::Mat camera_matrix_;
  // the keyframe, to which the images are registered until the camera moves keyframe_distance or keyframe_angle,
  // and the poses of the keyframe and of the previous image, in the frame of the camera at the first image
  cv::Ptr<cv::rgbd::OdometryFrame> keyframe_;
  cv::Matx44d keyframe_pose_;
  cv::Matx44d previous_pose_;
  cv::Matx44d previous_motion_;  // for the initial guess of the constant velocity
  double keyframe_distance_;
  double keyframe_angle_;

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
      else if (parameter.get_name() == "keyframe_distance")
        keyframe_distance_ = parameter.as_double();
      else if (parameter.get_name() == "keyframe_angle")
        keyframe_angle_ = parameter.as_double();
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
    keyframe_.release();  // the cache of the frame depends on the odometry
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

  // the camera matrix of camera_info of the rectified image and depth (P, or K without P), adjusted to the image
  // cropped by roi and binned (0 means 1), as aruco_detection; the images are rectified (e.g. image_rect and
  // aligned_depth_to_color of realsense2_camera, whose distortion is 0), as the odometry is of the pinhole model
  cv::Mat cameraMatrix(const sensor_msgs::msg::CameraInfo& info)
  {
// the fields of sensor_msgs/CameraInfo are lower case on ROS2
#ifdef ROS2
    const double* p = info.p.data();
    const std::vector<double>& d = info.d;
#else
    const double* p = info.P.data();
    const std::vector<double>& d = info.D;
#endif
    if (std::any_of(d.begin(), d.end(), [](double c) { return c != 0.0; }))
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000,
                           "camera_info has the distortion; give the rectified image and depth, of P of camera_info");
    // fx, fy, cx and cy of P, or of K if P is 0
    double fx = p[0], fy = p[5], cx = p[2], cy = p[6];
    if (fx == 0.0 || fy == 0.0)
    {
#ifdef ROS2
      const double* k = info.k.data();
#else
      const double* k = info.K.data();
#endif
      fx = k[0];
      fy = k[4];
      cx = k[2];
      cy = k[5];
    }
    const double k[9] = { fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 };
    double binning_x = std::max(1u, static_cast<unsigned int>(info.binning_x));
    double binning_y = std::max(1u, static_cast<unsigned int>(info.binning_y));
    // cv::Mat of cv::Matx, as OpenCV 5.1 has no comma initializer of cv::Mat_
    return cv::Mat(cv::Matx33d(k[0] / binning_x, 0.0, (k[2] - info.roi.x_offset) / binning_x, 0.0, k[4] / binning_y,
                               (k[5] - info.roi.y_offset) / binning_y, 0.0, 0.0, 1.0),
                   true);
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
      std_msgs::msg::Header header = depth_msg->header;
      if (!info_msg->header.frame_id.empty())
        header.frame_id = info_msg->header.frame_id;
      if (!sensorOnBase(header.frame_id))
        return;
      cv::Mat camera_matrix = cameraMatrix(*info_msg);
      // the rectified frame of P in the optical frame of the camera (header.frame_id): R of camera_info, the identity
      // if R is 0; the motion and the points of the rectified frame are rotated back to the optical frame
#ifdef ROS2
      const double* r = info_msg->r.data();
#else
      const double* r = info_msg->R.data();
#endif
      cv::Matx44d rectified = cv::Matx44d::eye();
      if (std::any_of(r, r + 9, [](double c) { return c != 0.0; }))
        rectified = cv::Matx44d(r[0], r[1], r[2], 0, r[3], r[4], r[5], 0, r[6], r[7], r[8], 0, 0, 0, 0, 1);
      if (camera_matrix.at<double>(0, 0) == 0.0 || !updateOdometry(camera_matrix))
        return;
      cv::Mat depth = depthInMeters(depth_msg);
      // a black image without the image (use_image false), for icp, which uses the depth only
      cv::Mat gray;
      cv::Mat image =
          image_msg ? cv_bridge::toCvShare(image_msg)->image : cv::Mat(depth.size(), CV_8UC1, cv::Scalar(0));
      if (image.channels() == 3)
        cv::cvtColor(image, gray,
                     image_msg->encoding == sensor_msgs::image_encodings::RGB8 ? cv::COLOR_RGB2GRAY :
                                                                                 cv::COLOR_BGR2GRAY);
      else if (image.channels() == 4)
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
      else
        gray = image.clone();
      if (gray.size() != depth.size() || gray.depth() != CV_8U)
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                              "the image (8 bit) and the depth image registered to it must be of the same size");
        return;
      }
      // restart the odometry at the first image, or when the time goes back, e.g. a bag played in a loop
      if (restartOdometry(rclcpp::Time(header.stamp)))
      {
        keyframe_.release();
        previous_pose_ = previous_motion_ = cv::Matx44d::eye();
      }
      cv::Ptr<cv::rgbd::OdometryFrame> frame = cv::rgbd::OdometryFrame::create(gray, depth);
      odometry_->prepareFrameCache(frame, cv::rgbd::OdometryFrame::CACHE_ALL);
      cv::Matx44d motion = cv::Matx44d::eye();  // of the camera from the previous image to this image
      // the first image, or the first image after the odometry is recreated (its frames have the caches of it)
      if (!keyframe_)
      {
        keyframe_ = frame;
        keyframe_pose_ = previous_pose_;
        previous_motion_ = cv::Matx44d::eye();
      }
      else
      {
        // Rt transforms the points of the keyframe into this image, the inverse of the camera in the keyframe; the
        // initial guess of the constant velocity
        cv::Matx44d guess = previous_pose_ * previous_motion_;
        cv::Mat rt, init(keyframe_pose_.inv() * guess);
        init = init.inv();
        cv::Matx44d pose = guess;
        // the result of compute(), as rt may be left even when it fails
        bool registered = odometry_->compute(keyframe_, frame, rt, init);
        if (registered)
          pose = keyframe_pose_ * cv::Matx44d(rt).inv();
        else
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                               "the odometry failed, as the constant velocity, and a new keyframe");
        motion = previous_pose_.inv() * pose;
        // a new keyframe when the camera moves enough from the keyframe, or when the odometry fails
        cv::Matx44d from_keyframe = keyframe_pose_.inv() * pose;
        cv::Vec3d axis_angle;
        cv::Rodrigues(cv::Matx33d(from_keyframe.get_minor<3, 3>(0, 0)), axis_angle);
        if (!registered ||
            cv::norm(cv::Vec3d(from_keyframe(0, 3), from_keyframe(1, 3), from_keyframe(2, 3))) > keyframe_distance_ ||
            cv::norm(axis_angle) > keyframe_angle_)
        {
          keyframe_ = frame;
          keyframe_pose_ = pose;
        }
        previous_pose_ = pose;
        previous_motion_ = motion;
      }
      publishOdometry(header, rectified.t() * motion * rectified);

      // the colored points of every 4th pixel to the 3D map
      std::vector<cv::Point3d> points;
      std::vector<cv::Vec3b> colors;
      const double fx = camera_matrix.at<double>(0, 0), fy = camera_matrix.at<double>(1, 1);
      const double cx = camera_matrix.at<double>(0, 2), cy = camera_matrix.at<double>(1, 2);
      for (int v = 0; v < depth.rows; v += 4)
        for (int u = 0; u < depth.cols; u += 4)
        {
          float z = depth.at<float>(v, u);
          if (!std::isfinite(z) || z < min_depth_ || z > max_depth_)
            continue;
          cv::Vec4d point = rectified.t() * cv::Vec4d((u - cx) * z / fx, (v - cy) * z / fy, z, 1.0);
          points.push_back(cv::Point3d(point[0], point[1], point[2]));
          if (!image_msg)
            colors.push_back(cv::Vec3b::all(255));
          else if (image.channels() == 1)
            colors.push_back(cv::Vec3b::all(image.at<uint8_t>(v, u)));
          else if (image_msg->encoding == sensor_msgs::image_encodings::RGB8)
          {
            const cv::Vec3b& rgb = image.at<cv::Vec3b>(v, u);
            colors.push_back(cv::Vec3b(rgb[2], rgb[1], rgb[0]));
          }
          else
            colors.push_back(cv::Vec3b(image.ptr<uint8_t>(v)[u * image.channels()],
                                       image.ptr<uint8_t>(v)[u * image.channels() + 1],
                                       image.ptr<uint8_t>(v)[u * image.channels() + 2]));
        }
      addToMap(header, points, colors);
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

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
  {
    callback(sensor_msgs::msg::Image::ConstSharedPtr(), depth_msg, info_msg);
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "subscribe");
    // image_transport takes rclcpp::QoS since lyrical
#if __has_include(<image_transport/node_interfaces.hpp>)
    if (use_image_)
      image_sub_.subscribe(*this, "image", image_transport::TransportHints(*this).getTransport(),
                           subscriberQos(queue_size_));
    depth_sub_.subscribe(*this, "depth", "raw", subscriberQos(queue_size_));
#else
    if (use_image_)
      image_sub_.subscribe(this, "image", image_transport::TransportHints(this).getTransport(),
                           subscriberQos(queue_size_).get_rmw_qos_profile());
    depth_sub_.subscribe(this, "depth", "raw", subscriberQos(queue_size_).get_rmw_qos_profile());
#endif
    subscribeFilter(info_sub_, "camera_info");
    if (!use_image_)
    {
      depth_sync_ = std::make_shared<DepthSync>(DepthSyncPolicy(queue_size_));
      depth_sync_->connectInput(depth_sub_, info_sub_);
      depth_sync_->registerCallback(
          std::bind(&RGBDOdometryNodelet::depthCallback, this, std::placeholders::_1, std::placeholders::_2));
      return;
    }
    sync_ = std::make_shared<Sync>(SyncPolicy(queue_size_));
    sync_->connectInput(image_sub_, depth_sub_, info_sub_);
    sync_->registerCallback(std::bind(&RGBDOdometryNodelet::callback, this, std::placeholders::_1,
                                      std::placeholders::_2, std::placeholders::_3));
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "unsubscribe");
    sync_.reset();
    depth_sync_.reset();
    image_sub_.unsubscribe();
    depth_sub_.unsubscribe();
    info_sub_.unsubscribe();
  }

public:
  explicit RGBDOdometryNodelet(const rclcpp::NodeOptions& options)
    : OdometryNodelet("rgbd_odometry", options)
    , need_odometry_update_(true)
    , keyframe_pose_(cv::Matx44d::eye())
    , previous_pose_(cv::Matx44d::eye())
    , previous_motion_(cv::Matx44d::eye())
  {
    queue_size_ = declare_parameter("queue_size", 10);
    method_ = declare_parameter(
        "method", std::string("rgbd_icp"),
        paramDescriptor("Method of the odometry: rgbd (the photometric error of the image, cv::rgbd::RgbdOdometry), "
                        "icp (the geometric error of the depth, cv::rgbd::ICPOdometry) or rgbd_icp (both, "
                        "cv::rgbd::RgbdICPOdometry)"));
    use_image_ = declare_parameter(
        "use_image", true,
        paramDescriptor("Subscribe the image; false for a depth camera without the image, of method icp"));
    if (!use_image_ && method_ != "icp")
    {
      RCLCPP_WARN(get_logger(), "method %s needs the image, use icp without the image (use_image false)",
                  method_.c_str());
      method_ = "icp";
    }
    min_depth_ = declare_parameter("min_depth", 0.3, paramDescriptor("Minimum valid depth (m).", 0.0, 100.0));
    max_depth_ = declare_parameter("max_depth", 4.0, paramDescriptor("Maximum valid depth (m).", 0.0, 1000.0));
    keyframe_distance_ = declare_parameter(
        "keyframe_distance", 0.1,
        paramDescriptor("The images are registered to the keyframe until the camera moves this distance (m) from it.",
                        0.0, 10.0));
    keyframe_angle_ = declare_parameter(
        "keyframe_angle", 0.1,
        paramDescriptor("The images are registered to the keyframe until the camera turns this angle (rad) from it.",
                        0.0, 3.2));
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
