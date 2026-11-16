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
 * e.g. for slam_toolbox and Nav2, and the 3D map (~/map, sensor_msgs/PointCloud2 of a point of each voxel of
 * map_voxel_size in odom_frame, with the colors of the RGB-D camera) of the points given by the derived class.
 */

#ifndef OPENCV_APPS_ODOMETRY_NODELET_H_
#define OPENCV_APPS_ODOMETRY_NODELET_H_

#include "opencv_apps/nodelet.h"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  int path_length_;
  double map_voxel_size_;
  double map_period_;

  cv::Matx44d pose_;         // the sensor in the frame of the sensor at the first input
  cv::Matx44d base_sensor_;  // the sensor on base_frame, by TF at the first input
  bool has_base_sensor_;
  rclcpp::Time previous_stamp_;
  bool has_previous_;
  nav_msgs::msg::Path path_;
  // the 3D map: a point (x, y, z and the color in BGR) of each voxel, in odom_frame of the robot
  std::unordered_map<int64_t, cv::Vec4f> map_;
  double map_stamp_ = 0.0;

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
    map_voxel_size_ = declare_parameter(
        "map_voxel_size", 0.05,
        paramDescriptor("Size (m) of the voxels of the 3D map (~/map, a point of each voxel), 0 for no map.", 0.0,
                        10.0));
    map_period_ =
        declare_parameter("map_period", 1.0, paramDescriptor("Period (s) to publish the 3D map (~/map).", 0.0, 3600.0));
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
    map_pub_ = advertise<sensor_msgs::msg::PointCloud2>("~/map", 1);
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
    map_.clear();
    has_previous_ = false;
    return true;
  }

  // add the points of the sensor (in the frame of the sensor, of the last publishOdometry()) to the 3D map, with
  // their colors (BGR, 8 bit) or without them (empty), and publish the map every map_period
  void addToMap(const std_msgs::msg::Header& header, const std::vector<cv::Point3d>& points,
                const std::vector<cv::Vec3b>& colors)
  {
    if (map_voxel_size_ <= 0.0)
      return;
    // the points in odom_frame, which is base_frame at the first input
    cv::Matx44d sensor = base_sensor_ * pose_;
    const int64_t mask = (1 << 21) - 1;
    for (size_t i = 0; i < points.size(); i++)
    {
      const cv::Point3d& p = points[i];
      cv::Vec3d q(sensor(0, 0) * p.x + sensor(0, 1) * p.y + sensor(0, 2) * p.z + sensor(0, 3),
                  sensor(1, 0) * p.x + sensor(1, 1) * p.y + sensor(1, 2) * p.z + sensor(1, 3),
                  sensor(2, 0) * p.x + sensor(2, 1) * p.y + sensor(2, 2) * p.z + sensor(2, 3));
      int64_t key = 0;
      for (int k = 0; k < 3; k++)
        key = (key << 21) | (static_cast<int64_t>(std::floor(q[k] / map_voxel_size_)) & mask);
      // the color as the float of the packed rgb of PointCloud2
      uint32_t rgb = 0xffffff;
      if (!colors.empty())
        rgb = (static_cast<uint32_t>(colors[i][2]) << 16) | (static_cast<uint32_t>(colors[i][1]) << 8) | colors[i][0];
      float packed;
      std::memcpy(&packed, &rgb, sizeof(float));
      map_.emplace(key, cv::Vec4f(static_cast<float>(q[0]), static_cast<float>(q[1]), static_cast<float>(q[2]), packed));
    }
    // the seconds, as rclcpp::Time of the different clocks cannot be compared
    double stamp = rclcpp::Time(header.stamp).seconds();
    if (stamp - map_stamp_ < map_period_ && stamp >= map_stamp_)
      return;
    map_stamp_ = stamp;
    sensor_msgs::msg::PointCloud2 map_msg;
    map_msg.header.stamp = header.stamp;
    map_msg.header.frame_id = odom_frame_;
    map_msg.height = 1;
    map_msg.width = static_cast<uint32_t>(map_.size());
    const char* names[4] = { "x", "y", "z", "rgb" };
    for (int k = 0; k < 4; k++)
    {
      sensor_msgs::msg::PointField field;
      field.name = names[k];
      field.offset = 4 * k;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      map_msg.fields.push_back(field);
    }
    map_msg.is_bigendian = false;
    map_msg.point_step = 16;
    map_msg.row_step = 16 * map_msg.width;
    map_msg.is_dense = true;
    map_msg.data.resize(map_msg.row_step);
    size_t offset = 0;
    for (const auto& voxel : map_)
    {
      std::memcpy(&map_msg.data[offset], voxel.second.val, 16);
      offset += 16;
    }
    map_pub_->publish(map_msg);
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
      // the velocity in the current base frame (child_frame_id): the displacement in the previous base frame rotated
      // into the current one; the axis of the rotation is the same in both
      cv::Matx33d rotation = base_motion.get_minor<3, 3>(0, 0);
      cv::Vec3d axis_angle;
      cv::Rodrigues(rotation, axis_angle);
      cv::Vec3d velocity = rotation.t() * cv::Vec3d(base_motion(0, 3), base_motion(1, 3), base_motion(2, 3)) / dt;
      odom_msg.twist.twist.linear.x = velocity[0];
      odom_msg.twist.twist.linear.y = velocity[1];
      odom_msg.twist.twist.linear.z = velocity[2];
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
