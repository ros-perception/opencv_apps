// ROS1 compat of <vision_msgs/msg/detection3_d_array.hpp>, see opencv_apps/ros1_compat.h
// vision_msgs of ROS1 (0.0.2) has the class id (int64 id) in ObjectHypothesisWithPose, and of ROS2 (4) the label
// (hypothesis.class_id) and the id of the object (Detection3D::id); the nodes write the fields with #ifdef ROS2
#ifndef OPENCV_APPS_ROS1_VISION_MSGS_MSG_DETECTION3_D_ARRAY_HPP_
#define OPENCV_APPS_ROS1_VISION_MSGS_MSG_DETECTION3_D_ARRAY_HPP_

#include <vision_msgs/Detection3DArray.h>
#include "opencv_apps/ros1_compat.h"

namespace vision_msgs
{
namespace msg = ::vision_msgs;
}

#endif  // OPENCV_APPS_ROS1_VISION_MSGS_MSG_DETECTION3_D_ARRAY_HPP_
