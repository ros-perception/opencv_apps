// ROS1 compat of <tf2_ros/transform_broadcaster.hpp>, see opencv_apps/ros1_compat.h
// Within namespace opencv_apps, tf2_ros::TransformBroadcaster also has the constructor of ROS2, which takes the node.
#ifndef OPENCV_APPS_ROS1_TF2_ROS_TRANSFORM_BROADCASTER_HPP_
#define OPENCV_APPS_ROS1_TF2_ROS_TRANSFORM_BROADCASTER_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include "opencv_apps/ros1_compat.h"

namespace opencv_apps
{
namespace tf2_ros
{
class TransformBroadcaster : public ::tf2_ros::TransformBroadcaster
{
public:
  template <class NodeT>
  explicit TransformBroadcaster(NodeT& /*node*/)
  {
  }
};
}  // namespace tf2_ros
}  // namespace opencv_apps

#endif  // OPENCV_APPS_ROS1_TF2_ROS_TRANSFORM_BROADCASTER_HPP_
