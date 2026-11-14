// ROS1 compat of <tf2_ros/transform_listener.hpp>, see opencv_apps/ros1_compat.h
#ifndef OPENCV_APPS_ROS1_TF2_ROS_TRANSFORM_LISTENER_HPP_
#define OPENCV_APPS_ROS1_TF2_ROS_TRANSFORM_LISTENER_HPP_

#include <tf2_ros/transform_listener.h>
#include "opencv_apps/ros1_compat.h"

// tf2_ros:: within namespace opencv_apps is opencv_apps::tf2_ros of <tf2_ros/transform_broadcaster.hpp>
namespace opencv_apps
{
namespace tf2_ros
{
using ::tf2_ros::TransformListener;
}  // namespace tf2_ros
}  // namespace opencv_apps

#endif  // OPENCV_APPS_ROS1_TF2_ROS_TRANSFORM_LISTENER_HPP_
