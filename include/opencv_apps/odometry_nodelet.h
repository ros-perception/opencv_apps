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


/**
 * Base class of the odometry nodes, rgbd_odometry and laser_odometry: the derived class estimates the motion of its
 * sensor between the consecutive inputs, and this class accumulates it into the pose of the robot (base_frame, by the
 * TF of the sensor on it at the first input) in odom_frame, the robot at the first input, and publishes it as
 * nav_msgs/Odometry (~/odom), the trajectory (~/path, nav_msgs/Path for rviz) and the TF odom_frame -> base_frame,
 * e.g. for slam_toolbox and Nav2.
 */

#ifndef OPENCV_APPS_ODOMETRY_NODELET_H_
#define OPENCV_APPS_ODOMETRY_NODELET_H_

#include "opencv_apps/nodelet.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
// the .hpp of tf2_ros of ROS 2 (jazzy and later), or of the compat headers of ROS 1 (ros1/include)
#if !defined(ROS2) || __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#endif

namespace opencv_apps
{
class OdometryNodelet : public opencv_apps::Nodelet
{
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  int path_length_;

  cv::Matx44d pose_;         // the sensor in the frame of the sensor at the first input
  cv::Matx44d base_sensor_;  // the sensor on base_frame, by TF at the first input
  bool has_base_sensor_;
  rclcpp::Time previous_stamp_;
  bool has_previous_;
  nav_msgs::msg::Path path_;

protected:
  OdometryNodelet(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Nodelet(node_name, options)
    , pose_(cv::Matx44d::eye())
    , base_sensor_(cv::Matx44d::eye())
    , has_base_sensor_(false)
    , has_previous_(false)
  {
  }

  // declare the parameters of the odometry and advertise its outputs, called in the constructor of the derived class
  void initOdometry()
  {
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"),
                                    paramDescriptor("Frame of the odometry, the robot (base_frame) at the start"));
    base_frame_ = declare_parameter(
        "base_frame", std::string(""),
        paramDescriptor("Frame of the robot, whose TF to the sensor is given (e.g. base_link, base_footprint), or "
                        "empty for the frame of the sensor"));
    publish_tf_ = declare_parameter(
        "publish_tf", true,
        paramDescriptor("Publish the TF odom_frame -> base_frame; with it at the start, the node subscribes the "
                        "inputs always, for the subscribers of /tf."));
    path_length_ = declare_parameter(
        "path_length", 10000,
        paramDescriptor("Number of the poses of ~/path, the trajectory for rviz, 0 for all of them.", 0, 1000000));
    // the TF is published only when the inputs are subscribed, and a subscriber of /tf (e.g. slam_toolbox) does not
    // subscribe the outputs of the node, so the node always subscribes the inputs with publish_tf at the start
    if (publish_tf_)
      always_subscribe_ = true;
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(*this));
#ifdef ROS2
    tf_buffer_.reset(new tf2_ros::Buffer(get_clock()));
#else
    tf_buffer_.reset(new tf2_ros::Buffer());
#endif
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    odom_pub_ = advertise<nav_msgs::msg::Odometry>("~/odom", 1);
    path_pub_ = advertise<nav_msgs::msg::Path>("~/path", 1);
  }

  static geometry_msgs::msg::Pose toPose(const cv::Matx44d& transform)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform(0, 3);
    pose.position.y = transform(1, 3);
    pose.position.z = transform(2, 3);
    // the quaternion of the axis and the angle of the rotation
    cv::Vec3d axis_angle;
    cv::Rodrigues(cv::Matx33d(transform.get_minor<3, 3>(0, 0)), axis_angle);
    double angle = cv::norm(axis_angle);
    cv::Vec3d axis = angle > 1e-12 ? axis_angle / angle : cv::Vec3d(0, 0, 0);
    pose.orientation.x = axis[0] * std::sin(angle / 2);
    pose.orientation.y = axis[1] * std::sin(angle / 2);
    pose.orientation.z = axis[2] * std::sin(angle / 2);
    pose.orientation.w = std::cos(angle / 2);
    return pose;
  }

  static cv::Matx44d toMatrix(const geometry_msgs::msg::Transform& transform)
  {
    const auto& q = transform.rotation;
    const auto& t = transform.translation;
    return cv::Matx44d(1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w), t.x,
                       2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w), t.y,
                       2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y), t.z,
                       0, 0, 0, 1);
  }

  // the sensor on base_frame by TF at the first input, the identity without base_frame, false if the transform is
  // not available yet
  bool sensorOnBase(const std::string& sensor_frame)
  {
    if (has_base_sensor_)
      return true;
    if (base_frame_.empty() || base_frame_ == sensor_frame)
    {
      base_sensor_ = cv::Matx44d::eye();
      has_base_sensor_ = true;
      return true;
    }
    try
    {
#ifdef ROS2
      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform(base_frame_, sensor_frame, tf2::TimePointZero);
#else
      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform(base_frame_, sensor_frame, ros::Time(0));
#endif
      base_sensor_ = toMatrix(transform.transform);
      has_base_sensor_ = true;
      return true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "no transform from %s to %s: %s", sensor_frame.c_str(),
                           base_frame_.c_str(), e.what());
      return false;
    }
  }

  // true if the odometry restarts at this input: the first input, or the time went back (e.g. a bag played in a loop)
  bool restartOdometry(const rclcpp::Time& stamp)
  {
    if (has_previous_ && stamp >= previous_stamp_)
      return false;
    if (has_previous_)
      RCLCPP_WARN(get_logger(), "the time went back, restart the odometry");
    pose_ = cv::Matx44d::eye();
    path_.poses.clear();
    has_previous_ = false;
    return true;
  }

  // accumulate the motion of the sensor (the sensor of this input in the sensor of the previous input, the identity at
  // the first input) and publish the odometry of the robot, after sensorOnBase()
  void publishOdometry(const std_msgs::msg::Header& header, const cv::Matx44d& motion)
  {
    rclcpp::Time stamp(header.stamp);
    double dt = has_previous_ ? stamp.seconds() - previous_stamp_.seconds() : 0.0;
    pose_ = pose_ * motion;
    previous_stamp_ = stamp;
    has_previous_ = true;

    // the robot in odom_frame, which is the robot at the first input
    cv::Matx44d base = base_sensor_ * pose_ * base_sensor_.inv();
    cv::Matx44d base_motion = base_sensor_ * motion * base_sensor_.inv();
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_.empty() ? header.frame_id : base_frame_;
    odom_msg.pose.pose = toPose(base);
    if (dt > 0.0)
    {
      cv::Vec3d axis_angle;
      cv::Rodrigues(cv::Matx33d(base_motion.get_minor<3, 3>(0, 0)), axis_angle);
      odom_msg.twist.twist.linear.x = base_motion(0, 3) / dt;
      odom_msg.twist.twist.linear.y = base_motion(1, 3) / dt;
      odom_msg.twist.twist.linear.z = base_motion(2, 3) / dt;
      odom_msg.twist.twist.angular.x = axis_angle[0] / dt;
      odom_msg.twist.twist.angular.y = axis_angle[1] / dt;
      odom_msg.twist.twist.angular.z = axis_angle[2] / dt;
    }
    // a fixed covariance, for robot_localization and the others which need it
    for (int i = 0; i < 6; i++)
    {
      odom_msg.pose.covariance[i * 7] = 1e-3;
      odom_msg.twist.covariance[i * 7] = 1e-2;
    }
    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom_msg.header;
    pose_msg.pose = odom_msg.pose.pose;
    path_.header = odom_msg.header;
    path_.poses.push_back(pose_msg);
    if (path_length_ > 0 && static_cast<int>(path_.poses.size()) > path_length_)
      path_.poses.erase(path_.poses.begin(), path_.poses.end() - path_length_);
    path_pub_->publish(path_);

    if (publish_tf_)
    {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom_msg.header;
      transform.child_frame_id = odom_msg.child_frame_id;
      transform.transform.translation.x = odom_msg.pose.pose.position.x;
      transform.transform.translation.y = odom_msg.pose.pose.position.y;
      transform.transform.translation.z = odom_msg.pose.pose.position.z;
      transform.transform.rotation = odom_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }
};
}  // namespace opencv_apps

#endif  // OPENCV_APPS_ODOMETRY_NODELET_H_
