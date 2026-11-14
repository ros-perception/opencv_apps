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
 * 3D bounding boxes and TF frames of the 2D detections, faces or tracked targets, from a depth image registered to
 * the color image of the detections (e.g. of an RGB-D camera): the depth of each box is the median of the depths in
 * the box, and the box is the extent of the pixels within depth_threshold of the median, as yolo_ros
 * (https://github.com/mgonzs13/yolo_ros). The objects are tracked by the distance of their centers, so that each
 * object keeps its TF frame, <label>_<id>.
 */

#include "opencv_apps/nodelet.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/version.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif

#include <opencv_apps/msg/detection_array_stamped.hpp>
#include <opencv_apps/msg/face_array_stamped.hpp>
#include <opencv_apps/msg/rect_array_stamped.hpp>

namespace opencv_apps
{
class Detection3DNodelet : public opencv_apps::Nodelet
{
  // a 2D box (center, size) with its label and score
  struct Box2D
  {
    double x, y, width, height;
    std::string label;
    int class_id;
    double score;
  };

  // a tracked object: its id, label and last position in the camera frame
  struct Track
  {
    int id;
    std::string label;
    cv::Point3d center;
    int lost;
  };

  template <class T>
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, T>;
  template <class T>
  using Sync = message_filters::Synchronizer<SyncPolicy<T> >;

  image_transport::SubscriberFilter depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  message_filters::Subscriber<opencv_apps::msg::DetectionArrayStamped> detections_sub_;
  message_filters::Subscriber<opencv_apps::msg::FaceArrayStamped> faces_sub_;
  message_filters::Subscriber<opencv_apps::msg::RectArrayStamped> rects_sub_;
  std::shared_ptr<Sync<opencv_apps::msg::DetectionArrayStamped> > detections_sync_;
  std::shared_ptr<Sync<opencv_apps::msg::FaceArrayStamped> > faces_sync_;
  std::shared_ptr<Sync<opencv_apps::msg::RectArrayStamped> > rects_sync_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  std::string input_type_;
  double roi_scale_;
  double depth_threshold_;
  double min_depth_;
  double max_depth_;
  bool publish_tf_;
  std::string tf_prefix_;
  double track_distance_;
  int track_lost_frames_;

  std::vector<Track> tracks_;
  std::map<std::string, int> next_ids_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "roi_scale")
        roi_scale_ = parameter.as_double();
      else if (parameter.get_name() == "depth_threshold")
        depth_threshold_ = parameter.as_double();
      else if (parameter.get_name() == "min_depth")
        min_depth_ = parameter.as_double();
      else if (parameter.get_name() == "max_depth")
        max_depth_ = parameter.as_double();
      else if (parameter.get_name() == "publish_tf")
        publish_tf_ = parameter.as_bool();
      else if (parameter.get_name() == "tf_prefix")
        tf_prefix_ = parameter.as_string();
      else if (parameter.get_name() == "track_distance")
        track_distance_ = parameter.as_double();
      else if (parameter.get_name() == "track_lost_frames")
        track_lost_frames_ = parameter.as_int();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void detectionsCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
                          const opencv_apps::msg::DetectionArrayStamped::ConstSharedPtr& msg)
  {
    std::vector<Box2D> boxes;
    for (const opencv_apps::msg::Detection& d : msg->detections)
      boxes.push_back(
          { d.rect.x, d.rect.y, d.rect.width, d.rect.height, d.label.empty() ? "object" : d.label, d.id, d.score });
    doWork(depth_msg, info_msg, boxes);
  }

  void facesCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
                     const opencv_apps::msg::FaceArrayStamped::ConstSharedPtr& msg)
  {
    std::vector<Box2D> boxes;
    for (const opencv_apps::msg::Face& f : msg->faces)
      boxes.push_back({ f.face.x, f.face.y, f.face.width, f.face.height, f.label.empty() ? "face" : f.label, -1,
                        f.confidence > 0.0 ? f.confidence : 1.0 });
    doWork(depth_msg, info_msg, boxes);
  }

  void rectsCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
                     const opencv_apps::msg::RectArrayStamped::ConstSharedPtr& msg)
  {
    std::vector<Box2D> boxes;
    for (const opencv_apps::msg::Rect& r : msg->rects)
      boxes.push_back({ r.x, r.y, r.width, r.height, "target", -1, 1.0 });
    doWork(depth_msg, info_msg, boxes);
  }

  // the depth image in meters
  static cv::Mat depthInMeters(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
  {
    cv::Mat depth = cv_bridge::toCvShare(depth_msg)->image;
    cv::Mat meters;
    if (depth.type() == CV_16UC1)
      depth.convertTo(meters, CV_32FC1, 0.001);  // millimeters, as openni and realsense
    else if (depth.type() == CV_32FC1)
      meters = depth;
    else
      throw std::runtime_error("depth image of encoding " + depth_msg->encoding + " is not 16UC1 nor 32FC1");
    return meters;
  }

  // the 3D box of a 2D box, false if the box has no valid depth
  bool box3D(const cv::Mat& depth, double fx, double fy, double cx, double cy, const Box2D& box, cv::Point3d& center,
             cv::Point3d& size) const
  {
    // the pixels in the center of the box (roi_scale), as the edges of the boxes often have the background
    cv::Rect roi(cvRound(box.x - box.width * roi_scale_ / 2), cvRound(box.y - box.height * roi_scale_ / 2),
                 cvRound(box.width * roi_scale_), cvRound(box.height * roi_scale_));
    roi &= cv::Rect(0, 0, depth.cols, depth.rows);
    if (roi.area() <= 0)
      return false;
    std::vector<float> depths;
    for (int v = roi.y; v < roi.y + roi.height; v++)
      for (int u = roi.x; u < roi.x + roi.width; u++)
      {
        float z = depth.at<float>(v, u);
        if (std::isfinite(z) && z >= min_depth_ && z <= max_depth_)
          depths.push_back(z);
      }
    if (depths.empty())
      return false;
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    double median = depths[depths.size() / 2];
    // the extent of the pixels near the median depth, which belong to the object
    cv::Point3d min_point(1e9, 1e9, 1e9), max_point(-1e9, -1e9, -1e9);
    int count = 0;
    for (int v = roi.y; v < roi.y + roi.height; v++)
      for (int u = roi.x; u < roi.x + roi.width; u++)
      {
        float z = depth.at<float>(v, u);
        if (!std::isfinite(z) || std::abs(z - median) > depth_threshold_)
          continue;
        cv::Point3d p((u - cx) * z / fx, (v - cy) * z / fy, z);
        min_point = cv::Point3d(std::min(min_point.x, p.x), std::min(min_point.y, p.y), std::min(min_point.z, p.z));
        max_point = cv::Point3d(std::max(max_point.x, p.x), std::max(max_point.y, p.y), std::max(max_point.z, p.z));
        count++;
      }
    if (count == 0)
      return false;
    center = (min_point + max_point) * 0.5;
    size = max_point - min_point;
    return true;
  }

  // the id of the track of an object, a new one if no track of the label is within track_distance
  int trackId(const std::string& label, const cv::Point3d& center, std::vector<bool>& matched)
  {
    int best = -1;
    double best_distance = track_distance_;
    for (size_t i = 0; i < tracks_.size(); i++)
    {
      if (matched[i] || tracks_[i].label != label)
        continue;
      double distance = cv::norm(tracks_[i].center - center);
      if (distance < best_distance)
      {
        best = static_cast<int>(i);
        best_distance = distance;
      }
    }
    if (best < 0)
    {
      tracks_.push_back({ next_ids_[label]++, label, center, 0 });
      matched.push_back(true);
      return tracks_.back().id;
    }
    tracks_[best].center = center;
    tracks_[best].lost = 0;
    matched[best] = true;
    return tracks_[best].id;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
              const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg, const std::vector<Box2D>& boxes)
  {
    try
    {
      cv::Mat depth = depthInMeters(depth_msg);
// the fields of sensor_msgs/CameraInfo are lower case on ROS2
#ifdef ROS2
      const double* k = info_msg->k.data();
#else
      const double* k = info_msg->K.data();
#endif
      if (k[0] == 0.0 || k[4] == 0.0)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "camera_info is not calibrated (K is 0)");
        return;
      }
      const std::string& frame_id =
          info_msg->header.frame_id.empty() ? depth_msg->header.frame_id : info_msg->header.frame_id;
      vision_msgs::msg::Detection3DArray detections_msg;
      detections_msg.header = depth_msg->header;
      detections_msg.header.frame_id = frame_id;
      std::vector<geometry_msgs::msg::TransformStamped> transforms;
      std::vector<bool> matched(tracks_.size(), false);
      for (const Box2D& box : boxes)
      {
        cv::Point3d center, size;
        if (!box3D(depth, k[0], k[4], k[2], k[5], box, center, size))
          continue;
        int id = trackId(box.label, center, matched);
        std::string name = box.label + "_" + std::to_string(id);

        vision_msgs::msg::Detection3D detection;
        detection.header = detections_msg.header;
        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
#ifdef ROS2
        // vision_msgs 4 of ROS2 has the label (class_id) and the id of the object
        hypothesis.hypothesis.class_id = box.label;
        hypothesis.hypothesis.score = box.score;
        detection.id = name;
#else
        // vision_msgs 0.0.2 of ROS1 has only the class id (of the detections)
        hypothesis.id = box.class_id;
        hypothesis.score = box.score;
#endif
        hypothesis.pose.pose.position.x = center.x;
        hypothesis.pose.pose.position.y = center.y;
        hypothesis.pose.pose.position.z = center.z;
        hypothesis.pose.pose.orientation.w = 1.0;
        detection.results.push_back(hypothesis);
        detection.bbox.center.position.x = center.x;
        detection.bbox.center.position.y = center.y;
        detection.bbox.center.position.z = center.z;
        detection.bbox.center.orientation.w = 1.0;
        detection.bbox.size.x = size.x;
        detection.bbox.size.y = size.y;
        detection.bbox.size.z = size.z;
        detections_msg.detections.push_back(detection);

        geometry_msgs::msg::TransformStamped transform;
        transform.header = detections_msg.header;
        transform.child_frame_id = tf_prefix_ + name;
        transform.transform.translation.x = center.x;
        transform.transform.translation.y = center.y;
        transform.transform.translation.z = center.z;
        transform.transform.rotation.w = 1.0;
        transforms.push_back(transform);
      }
      // forget the tracks lost for track_lost_frames images
      for (size_t i = 0; i < matched.size(); i++)
        if (!matched[i])
          tracks_[i].lost++;
      tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                   [this](const Track& track) { return track.lost > track_lost_frames_; }),
                    tracks_.end());

      detections_pub_->publish(detections_msg);
      if (publish_tf_ && !transforms.empty())
        tf_broadcaster_->sendTransform(transforms);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "subscribe");
    // image_transport takes rclcpp::QoS since lyrical, message_filters since kilted
#if __has_include(<image_transport/node_interfaces.hpp>)
    depth_sub_.subscribe(*this, "depth", "raw", subscriberQos(queue_size_));
#else
    depth_sub_.subscribe(this, "depth", "raw", subscriberQos(queue_size_).get_rmw_qos_profile());
#endif
#if RCLCPP_VERSION_GTE(29, 0, 0)
    info_sub_.subscribe(this, "camera_info", subscriberQos(queue_size_));
#else
    info_sub_.subscribe(this, "camera_info", subscriberQos(queue_size_).get_rmw_qos_profile());
#endif
    if (input_type_ == "faces")
      faces_sync_ = connect(faces_sub_, &Detection3DNodelet::facesCallback);
    else if (input_type_ == "rects")
      rects_sync_ = connect(rects_sub_, &Detection3DNodelet::rectsCallback);
    else
      detections_sync_ = connect(detections_sub_, &Detection3DNodelet::detectionsCallback);
  }

  template <class T>
  std::shared_ptr<Sync<T> >
  connect(message_filters::Subscriber<T>& sub,
          void (Detection3DNodelet::*callback)(const sensor_msgs::msg::Image::ConstSharedPtr&,
                                               const sensor_msgs::msg::CameraInfo::ConstSharedPtr&,
                                               const typename T::ConstSharedPtr&))
  {
#if RCLCPP_VERSION_GTE(29, 0, 0)
    sub.subscribe(this, "input", subscriberQos(queue_size_));
#else
    sub.subscribe(this, "input", subscriberQos(queue_size_).get_rmw_qos_profile());
#endif
    auto sync = std::make_shared<Sync<T> >(SyncPolicy<T>(queue_size_));
    sync->connectInput(depth_sub_, info_sub_, sub);
    sync->registerCallback(
        std::bind(callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    return sync;
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "unsubscribe");
    depth_sub_.unsubscribe();
    info_sub_.unsubscribe();
    detections_sub_.unsubscribe();
    faces_sub_.unsubscribe();
    rects_sub_.unsubscribe();
  }

public:
  explicit Detection3DNodelet(const rclcpp::NodeOptions& options) : Nodelet("detection_3d", options)
  {
    queue_size_ = declare_parameter("queue_size", 10);
    input_type_ = declare_parameter(
        "input_type", std::string("detections"),
        paramDescriptor("Type of input: detections (opencv_apps/DetectionArrayStamped, e.g. ~/detections of "
                        "dnn_object_detection), faces (FaceArrayStamped, ~/faces of the face nodes) or rects "
                        "(RectArrayStamped, ~/output of the tracking nodes)."));
    if (input_type_ != "detections" && input_type_ != "faces" && input_type_ != "rects")
    {
      RCLCPP_ERROR(get_logger(), "input_type %s is not detections, faces nor rects, use detections",
                   input_type_.c_str());
      input_type_ = "detections";
    }
    roi_scale_ = declare_parameter(
        "roi_scale", 0.8,
        paramDescriptor("Scale of the 2D box whose depths are used, smaller to leave out the background at the "
                        "edges of the box.",
                        0.1, 1.0));
    depth_threshold_ = declare_parameter(
        "depth_threshold", 0.3,
        paramDescriptor("The pixels within this distance (m) of the median depth of the box belong to the object.",
                        0.01, 10.0));
    min_depth_ = declare_parameter("min_depth", 0.1, paramDescriptor("Minimum valid depth (m).", 0.0, 100.0));
    max_depth_ = declare_parameter("max_depth", 10.0, paramDescriptor("Maximum valid depth (m).", 0.0, 1000.0));
    publish_tf_ = declare_parameter("publish_tf", true,
                                    paramDescriptor("Publish the TF frame <tf_prefix><label>_<id> of each object."));
    tf_prefix_ = declare_parameter("tf_prefix", std::string(""), paramDescriptor("Prefix of the TF frames."));
    track_distance_ = declare_parameter(
        "track_distance", 0.5,
        paramDescriptor("An object within this distance (m) of an object of the same label in the previous images is "
                        "the same object, and keeps its id.",
                        0.0, 10.0));
    track_lost_frames_ = declare_parameter(
        "track_lost_frames", 10, paramDescriptor("Number of images that a lost object keeps its id for.", 0, 1000));
    on_set_parameters_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&Detection3DNodelet::parametersCallback, this, std::placeholders::_1));

    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(*this));
    detections_pub_ = advertise<vision_msgs::msg::Detection3DArray>("~/detections", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::Detection3DNodelet)
