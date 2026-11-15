// ROS1 compat of <tf2_ros/buffer.hpp>, see opencv_apps/ros1_compat.h
#ifndef OPENCV_APPS_ROS1_TF2_ROS_BUFFER_HPP_
#define OPENCV_APPS_ROS1_TF2_ROS_BUFFER_HPP_

#include <tf2_ros/buffer.h>
#include "opencv_apps/ros1_compat.h"

// tf2_ros:: within namespace opencv_apps is opencv_apps::tf2_ros of <tf2_ros/transform_broadcaster.hpp>
namespace opencv_apps
{
namespace tf2_ros
{
using ::tf2_ros::Buffer;
}  // namespace tf2_ros
}  // namespace opencv_apps

#endif  // OPENCV_APPS_ROS1_TF2_ROS_BUFFER_HPP_
