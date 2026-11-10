// ROS1 compat of <rclcpp/version.h>, see opencv_apps/ros1_compat.h
#ifndef OPENCV_APPS_ROS1_RCLCPP_VERSION_H_
#define OPENCV_APPS_ROS1_RCLCPP_VERSION_H_

// older than any rclcpp, so that the ROS1 compat API (e.g. rmw_qos_profile_t) is used
#define RCLCPP_VERSION_GTE(major, minor, patch) 0

#endif  // OPENCV_APPS_ROS1_RCLCPP_VERSION_H_
