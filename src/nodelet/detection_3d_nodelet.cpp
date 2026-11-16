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
 * 3D bounding boxes and TF frames of the 2D detections, faces or tracked targets, with the distances from a depth
 * image registered to the color image of the detections (e.g. of an RGB-D camera, depth_source depth) or from a point
 * cloud (e.g. of a LiDAR, depth_source points), as yolo_ros (https://github.com/mgonzs13/yolo_ros): the points of a
 * 2D box are the pixels of the depth image in the box, or the points of the cloud projected into the box, the depth
 * of the box is the median of the depths of its points, and the 3D box is the extent of the points within
 * depth_threshold of the median. The objects are tracked by the distance of their centers, so that each object keeps
 * its TF frame, <label>_<id>.
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/version.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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

#include <opencv_apps/msg/detection_array_stamped.hpp>
#include <opencv_apps/msg/face_array_stamped.hpp>
#include <opencv_apps/msg/rect_array_stamped.hpp>
#include <opencv_apps/msg/skeleton_array_stamped.hpp>

namespace opencv_apps
{
namespace
{
// the bones of the skeletons of MediaPipe: the 33 landmarks of a person (dnn_pose_estimation) and the 21 landmarks
// of a hand (dnn_hand_pose_estimation)
const int POSE_BONES[][2] = { { 0, 1 },   { 1, 2 },   { 2, 3 },   { 3, 7 },   { 0, 4 },   { 4, 5 },   { 5, 6 },
                              { 6, 8 },   { 9, 10 },  { 11, 12 }, { 11, 13 }, { 13, 15 }, { 15, 17 }, { 15, 19 },
                              { 15, 21 }, { 17, 19 }, { 12, 14 }, { 14, 16 }, { 16, 18 }, { 16, 20 }, { 16, 22 },
                              { 18, 20 }, { 11, 23 }, { 12, 24 }, { 23, 24 }, { 23, 25 }, { 24, 26 }, { 25, 27 },
                              { 26, 28 }, { 27, 29 }, { 28, 30 }, { 29, 31 }, { 30, 32 }, { 27, 31 }, { 28, 32 } };
const int HAND_BONES[][2] = { { 0, 1 },   { 1, 2 },   { 2, 3 },   { 3, 4 },   { 0, 5 },   { 5, 6 },   { 6, 7 },
                              { 7, 8 },   { 5, 9 },   { 9, 10 },  { 10, 11 }, { 11, 12 }, { 9, 13 },  { 13, 14 },
                              { 14, 15 }, { 15, 16 }, { 13, 17 }, { 0, 17 },  { 17, 18 }, { 18, 19 }, { 19, 20 } };
}  // namespace

class Detection3DNodelet : public opencv_apps::Nodelet
{
  // a 2D box (center, size) with its label and score
  // a keypoint of a skeleton: its index in the skeleton, name and position in the image
  struct Keypoint2D
  {
    int index;
    std::string name;
    cv::Point2d position;
  };

  // a 2D box (center, size) with its label and score, and the keypoints of a skeleton
  struct Box2D
  {
    double x, y, width, height;
    std::string label;
    int class_id;
    double score;
    int num_keypoints;  // the number of the keypoints of the skeleton, 0 for the other inputs
    std::vector<Keypoint2D> keypoints;
  };

  // a keypoint in 3D, in the camera frame
  struct Keypoint3D
  {
    int index;
    std::string name;
    cv::Point3d position;
  };

  // the camera of camera_info: the raw image with the distortion (K and D), or the rectified image (P) without it,
  // as image_geometry, adjusted to the image cropped by roi and binned; the 3D points are in the optical frame of the
  // camera, and the depths of the image in the frame of the image, rotated by R from the optical frame if rectified
  struct CameraModel
  {
    cv::Matx33d k;
    cv::Mat d;  // empty without the distortion
    bool fisheye;
    cv::Matx33d r;  // the rotation of the rectified image (R), the identity for the raw image
    int width, height;

    // the point of the optical frame in the frame of the image
    cv::Point3d toImage(const cv::Point3d& p) const
    {
      return cv::Point3d(r * cv::Vec3d(p.x, p.y, p.z));
    }

    // the 3D points of the pixels at their depths (in the frame of the image), in the optical frame
    std::vector<cv::Point3d> backProject(const std::vector<cv::Point2d>& pixels, const std::vector<double>& depths) const
    {
      std::vector<cv::Point2d> rays;
      if (d.empty())
        for (const cv::Point2d& pixel : pixels)
          rays.push_back(cv::Point2d((pixel.x - k(0, 2)) / k(0, 0), (pixel.y - k(1, 2)) / k(1, 1)));
      else if (!pixels.empty() && fisheye)
        cv::fisheye::undistortPoints(pixels, rays, k, d);
      else if (!pixels.empty())
        cv::undistortPoints(pixels, rays, k, d);
      std::vector<cv::Point3d> points;
      for (size_t i = 0; i < rays.size(); i++)
        points.push_back(cv::Point3d(r.t() * cv::Vec3d(rays[i].x * depths[i], rays[i].y * depths[i], depths[i])));
      return points;
    }

    // the pixels of the 3D points of the optical frame, (-1, -1) for the points behind the image
    std::vector<cv::Point2d> project(const std::vector<cv::Point3d>& points) const
    {
      std::vector<cv::Point2d> pixels;
      if (d.empty())
        for (const cv::Point3d& point : points)
        {
          cv::Point3d p = toImage(point);
          pixels.push_back(p.z > 0.0 ? cv::Point2d(k(0, 0) * p.x / p.z + k(0, 2), k(1, 1) * p.y / p.z + k(1, 2)) :
                                       cv::Point2d(-1.0, -1.0));
        }
      else if (!points.empty() && fisheye)
        cv::fisheye::projectPoints(points, pixels, cv::Vec3d(), cv::Vec3d(), k, d);
      else if (!points.empty())
        cv::projectPoints(points, cv::Vec3d(), cv::Vec3d(), k, d, pixels);
      return pixels;
    }
  };

  // a tracked object: its id, label and last position in the camera frame
  struct Track
  {
    int id;
    std::string label;
    cv::Point3d center;
    int lost;
  };

  template <class D, class T>
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<D, sensor_msgs::msg::CameraInfo, T>;
  template <class D, class T>
  using Sync = message_filters::Synchronizer<SyncPolicy<D, T> >;

  image_transport::SubscriberFilter depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> points_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  message_filters::Subscriber<opencv_apps::msg::DetectionArrayStamped> detections_sub_;
  message_filters::Subscriber<opencv_apps::msg::FaceArrayStamped> faces_sub_;
  message_filters::Subscriber<opencv_apps::msg::RectArrayStamped> rects_sub_;
  message_filters::Subscriber<opencv_apps::msg::SkeletonArrayStamped> skeletons_sub_;
  // the synchronizer of the depth source and the input, one of them is used
  std::shared_ptr<void> sync_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  std::string input_type_;
  std::string depth_source_;
  double roi_scale_;
  double depth_threshold_;
  double min_depth_;
  double max_depth_;
  bool publish_tf_;
  bool rectified_;
  std::string tf_prefix_;
  double track_distance_;
  int track_lost_frames_;
  int keypoint_radius_;
  double keypoint_visibility_;
  std::string face_confidence_;

  // the parameters and the tracks, set by parametersCallback and read by the callbacks of the inputs, which may run
  // in other threads (e.g. the multithreaded nodelet manager of ROS 1)
  std::mutex mutex_;
  std::vector<Track> tracks_;
  std::map<std::string, int> next_ids_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
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
      else if (parameter.get_name() == "rectified")
        rectified_ = parameter.as_bool();
      else if (parameter.get_name() == "tf_prefix")
        tf_prefix_ = parameter.as_string();
      else if (parameter.get_name() == "track_distance")
        track_distance_ = parameter.as_double();
      else if (parameter.get_name() == "track_lost_frames")
        track_lost_frames_ = parameter.as_int();
      else if (parameter.get_name() == "keypoint_radius")
        keypoint_radius_ = parameter.as_int();
      else if (parameter.get_name() == "keypoint_visibility")
        keypoint_visibility_ = parameter.as_double();
      else if (parameter.get_name() == "face_confidence")
        face_confidence_ = parameter.as_string();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // the 2D boxes of the inputs
  static std::vector<Box2D> boxes(const opencv_apps::msg::DetectionArrayStamped& msg)
  {
    std::vector<Box2D> boxes;
    for (const opencv_apps::msg::Detection& d : msg.detections)
      boxes.push_back(
          { d.rect.x, d.rect.y, d.rect.width, d.rect.height, d.label.empty() ? "object" : d.label, d.id, d.score });
    return boxes;
  }

  // the score in [0, 1] (higher is better) of the confidence of a face, a probability (e.g. dnn_face_detection) or
  // a distance (face_recognition, smaller is better) by face_confidence; 1 without the confidence (0, face_detection)
  double faceScore(double confidence) const
  {
    if (face_confidence_ == "distance")
      return 1.0 / (1.0 + std::max(0.0, confidence));
    return confidence > 0.0 ? std::min(1.0, confidence) : 1.0;
  }

  std::vector<Box2D> boxes(const opencv_apps::msg::FaceArrayStamped& msg) const
  {
    std::vector<Box2D> boxes;
    for (const opencv_apps::msg::Face& f : msg.faces)
      boxes.push_back({ f.face.x, f.face.y, f.face.width, f.face.height, f.label.empty() ? "face" : f.label, -1,
                        faceScore(f.confidence) });
    return boxes;
  }

  static std::vector<Box2D> boxes(const opencv_apps::msg::RectArrayStamped& msg)
  {
    std::vector<Box2D> boxes;
    for (const opencv_apps::msg::Rect& r : msg.rects)
      boxes.push_back({ r.x, r.y, r.width, r.height, "target", -1, 1.0 });
    return boxes;
  }

  std::vector<Box2D> boxes(const opencv_apps::msg::SkeletonArrayStamped& msg) const
  {
    std::vector<Box2D> boxes;
    for (const opencv_apps::msg::Skeleton& skeleton : msg.skeletons)
    {
      // the hands of dnn_hand_pose_estimation are left or right
      std::string label = skeleton.label.empty() ? "skeleton" : skeleton.label;
      if (label == "left" || label == "right")
        label += "_hand";
      Box2D box = { skeleton.rect.x,
                    skeleton.rect.y,
                    skeleton.rect.width,
                    skeleton.rect.height,
                    label,
                    -1,
                    skeleton.score,
                    static_cast<int>(skeleton.keypoints.size()),
                    {} };
      for (size_t k = 0; k < skeleton.keypoints.size(); k++)
      {
        const opencv_apps::msg::Keypoint& keypoint = skeleton.keypoints[k];
        if (keypoint.visibility >= keypoint_visibility_)
          box.keypoints.push_back({ static_cast<int>(k), keypoint.name, cv::Point2d(keypoint.x, keypoint.y) });
      }
      boxes.push_back(box);
    }
    return boxes;
  }

  // the median of the depths within min_depth and max_depth (0 is no depth), false if there is none
  bool medianDepth(std::vector<double>& depths, double& median) const
  {
    depths.erase(
        std::remove_if(depths.begin(), depths.end(),
                       [this](double z) { return !std::isfinite(z) || z <= 0.0 || z < min_depth_ || z > max_depth_; }),
        depths.end());
    if (depths.empty())
      return false;
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    median = depths[depths.size() / 2];
    return true;
  }

  // the pixels of a 2D box whose depths are used (roi_scale of the box, as the edges of the boxes often have the
  // background), within the image
  cv::Rect roi(const Box2D& box, int width, int height) const
  {
    cv::Rect roi(cvRound(box.x - box.width * roi_scale_ / 2), cvRound(box.y - box.height * roi_scale_ / 2),
                 cvRound(box.width * roi_scale_), cvRound(box.height * roi_scale_));
    return roi & cv::Rect(0, 0, width, height);
  }

  template <class T>
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
                     const typename T::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
      CameraModel camera;
      if (!cameraModel(*info_msg, camera))
        return;
      cv::Mat depth = depthInMeters(depth_msg);
      std::vector<Box2D> input = boxes(*msg);
      // the 3D points of the pixels of each box
      std::vector<std::vector<cv::Point3d> > points(input.size());
      for (size_t i = 0; i < input.size(); i++)
      {
        cv::Rect r = roi(input[i], depth.cols, depth.rows);
        std::vector<cv::Point2d> pixels;
        std::vector<double> depths;
        for (int v = r.y; v < r.y + r.height; v++)
          for (int u = r.x; u < r.x + r.width; u++)
          {
            float z = depth.at<float>(v, u);
            if (std::isfinite(z) && z > 0.0f)
            {
              pixels.push_back(cv::Point2d(u, v));
              depths.push_back(z);
            }
          }
        points[i] = camera.backProject(pixels, depths);
      }
      // the 3D keypoints: the median depth of the pixels within keypoint_radius of each keypoint
      std::vector<std::vector<Keypoint3D> > keypoints(input.size());
      for (size_t i = 0; i < input.size(); i++)
        for (const Keypoint2D& keypoint : input[i].keypoints)
        {
          cv::Rect window(cvRound(keypoint.position.x) - keypoint_radius_,
                          cvRound(keypoint.position.y) - keypoint_radius_, 2 * keypoint_radius_ + 1,
                          2 * keypoint_radius_ + 1);
          window &= cv::Rect(0, 0, depth.cols, depth.rows);
          std::vector<double> depths;
          for (int v = window.y; v < window.y + window.height; v++)
            for (int u = window.x; u < window.x + window.width; u++)
              depths.push_back(depth.at<float>(v, u));
          double z;
          if (medianDepth(depths, z))
            keypoints[i].push_back(
                { keypoint.index, keypoint.name, camera.backProject({ keypoint.position }, { z })[0] });
        }
      publish(depth_msg->header.stamp, frameId(*info_msg, depth_msg->header.frame_id), input, points, keypoints);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
  }

  template <class T>
  void pointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
                      const typename T::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
      CameraModel camera;
      if (!cameraModel(*info_msg, camera))
        return;
      // x, y and z of the points are float (FLOAT32), as most drivers of LiDARs and depth_image_proc, in the points
      // of point_step in the rows of row_step, which may have padding at their ends
      size_t offsets[3];
      const char* names[3] = { "x", "y", "z" };
      for (int i = 0; i < 3; i++)
      {
        const char* name = names[i];
        auto field = std::find_if(cloud_msg->fields.begin(), cloud_msg->fields.end(),
                                  [name](const sensor_msgs::msg::PointField& f) { return f.name == name; });
        if (field == cloud_msg->fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            field->offset + sizeof(float) > cloud_msg->point_step)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "the field %s of the points is not FLOAT32", name);
          return;
        }
        offsets[i] = field->offset;
      }
      if (static_cast<size_t>(cloud_msg->width) * cloud_msg->point_step > cloud_msg->row_step ||
          static_cast<size_t>(cloud_msg->height) * cloud_msg->row_step > cloud_msg->data.size())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "the size of the points (%u x %u) is not of their data",
                             cloud_msg->width, cloud_msg->height);
        return;
      }
      const std::string frame_id = frameId(*info_msg, cloud_msg->header.frame_id);
      // the cloud in the camera frame, by the latest transform (e.g. the static transform of the extrinsics)
      geometry_msgs::msg::TransformStamped transform;
      try
      {
#ifdef ROS2
        transform = tf_buffer_->lookupTransform(frame_id, cloud_msg->header.frame_id, tf2::TimePointZero);
#else
        transform = tf_buffer_->lookupTransform(frame_id, cloud_msg->header.frame_id, ros::Time(0));
#endif
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "no transform from %s to %s: %s",
                             cloud_msg->header.frame_id.c_str(), frame_id.c_str(), e.what());
        return;
      }
      const auto& q = transform.transform.rotation;
      const auto& t = transform.transform.translation;
      cv::Matx33d rotation(1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w),
                           2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w),
                           2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y));
      cv::Vec3d translation(t.x, t.y, t.z);

      std::vector<Box2D> input = boxes(*msg);
      std::vector<cv::Rect> rois;
      for (const Box2D& box : input)
        rois.push_back(roi(box, camera.width, camera.height));
      // the points projected into each box, and the depths of the points projected within keypoint_radius of each
      // keypoint
      std::vector<std::vector<cv::Point3d> > points(input.size());
      std::vector<std::vector<std::vector<double> > > keypoint_depths(input.size());
      for (size_t i = 0; i < input.size(); i++)
        keypoint_depths[i].resize(input[i].keypoints.size());
      // the points in front of the camera, in the camera frame
      std::vector<cv::Point3d> cloud;
      for (size_t row = 0; row < cloud_msg->height; row++)
        for (size_t column = 0; column < cloud_msg->width; column++)
        {
          const uint8_t* point = cloud_msg->data.data() + row * cloud_msg->row_step + column * cloud_msg->point_step;
          float xyz[3];
          for (int i = 0; i < 3; i++)
            std::memcpy(&xyz[i], point + offsets[i], sizeof(float));
          if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
            continue;
          cv::Vec3d p = rotation * cv::Vec3d(xyz[0], xyz[1], xyz[2]) + translation;
          if (p[2] > 0.0)
            cloud.push_back(cv::Point3d(p[0], p[1], p[2]));
        }
      std::vector<cv::Point2d> pixels = camera.project(cloud);
      for (size_t j = 0; j < cloud.size(); j++)
      {
        cv::Point pixel(cvRound(pixels[j].x), cvRound(pixels[j].y));
        for (size_t i = 0; i < rois.size(); i++)
        {
          if (rois[i].contains(pixel))
            points[i].push_back(cloud[j]);
          for (size_t k = 0; k < input[i].keypoints.size(); k++)
            if (std::abs(pixel.x - input[i].keypoints[k].position.x) <= keypoint_radius_ &&
                std::abs(pixel.y - input[i].keypoints[k].position.y) <= keypoint_radius_)
              keypoint_depths[i][k].push_back(camera.toImage(cloud[j]).z);
        }
      }
      std::vector<std::vector<Keypoint3D> > keypoints(input.size());
      for (size_t i = 0; i < input.size(); i++)
        for (size_t k = 0; k < input[i].keypoints.size(); k++)
        {
          const Keypoint2D& keypoint = input[i].keypoints[k];
          double z;
          if (medianDepth(keypoint_depths[i][k], z))
            keypoints[i].push_back(
                { keypoint.index, keypoint.name, camera.backProject({ keypoint.position }, { z })[0] });
        }
      publish(cloud_msg->header.stamp, frame_id, input, points, keypoints);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
  }

  // the camera model of camera_info, false if it is not calibrated: K and D of the raw image (rectified false), or P
  // and R of the rectified image (rectified true), whose camera_info is the same as the raw image; K and P are of the
  // full resolution image, adjusted to the image cropped by roi and binned (0 means 1), as aruco_detection
  bool cameraModel(const sensor_msgs::msg::CameraInfo& info, CameraModel& camera)
  {
// the fields of sensor_msgs/CameraInfo are lower case on ROS2
#ifdef ROS2
    const double* k = info.k.data();
    const double* p = info.p.data();
    const double* r = info.r.data();
    const std::vector<double>& d = info.d;
#else
    const double* k = info.K.data();
    const double* p = info.P.data();
    const double* r = info.R.data();
    const std::vector<double>& d = info.D;
#endif
    if (k[0] == 0.0 || k[4] == 0.0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "camera_info is not calibrated (K is 0)");
      return false;
    }
    // the fisheye model (equidistant) is not the pinhole model even with the coefficients of 0
    camera.fisheye = !rectified_ && info.distortion_model == "equidistant";
    bool distorted = camera.fisheye || std::any_of(d.begin(), d.end(), [](double c) { return c != 0.0; });
    camera.d = !rectified_ && distorted ? cv::Mat(d, true) : cv::Mat();
    if (camera.fisheye && camera.d.total() != 4)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "D of equidistant has %zu coefficients, not 4", d.size());
      return false;
    }
    // P and R of the rectified image (the translation of P, of the stereo cameras, is not used)
    camera.r = cv::Matx33d::eye();
    if (rectified_ && std::any_of(r, r + 9, [](double c) { return c != 0.0; }))
      camera.r = cv::Matx33d(r);
    double fx = k[0], fy = k[4], cx = k[2], cy = k[5];
    if (rectified_ && p[0] != 0.0 && p[5] != 0.0)
    {
      fx = p[0];
      fy = p[5];
      cx = p[2];
      cy = p[6];
    }
    double binning_x = std::max(1u, static_cast<unsigned int>(info.binning_x));
    double binning_y = std::max(1u, static_cast<unsigned int>(info.binning_y));
    camera.k = cv::Matx33d(fx / binning_x, 0.0, (cx - info.roi.x_offset) / binning_x, 0.0, fy / binning_y,
                           (cy - info.roi.y_offset) / binning_y, 0.0, 0.0, 1.0);
    camera.width = static_cast<int>((info.roi.width > 0 ? info.roi.width : info.width) / binning_x);
    camera.height = static_cast<int>((info.roi.height > 0 ? info.roi.height : info.height) / binning_y);
    return true;
  }

  static std::string frameId(const sensor_msgs::msg::CameraInfo& info, const std::string& fallback)
  {
    return info.header.frame_id.empty() ? fallback : info.header.frame_id;
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

  // the 3D box of the points of a 2D box, false if it has no point within min_depth and max_depth
  bool box3D(const std::vector<cv::Point3d>& points, cv::Point3d& center, cv::Point3d& size) const
  {
    std::vector<double> depths;
    for (const cv::Point3d& p : points)
      if (p.z >= min_depth_ && p.z <= max_depth_)
        depths.push_back(p.z);
    if (depths.empty())
      return false;
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    double median = depths[depths.size() / 2];
    // the extent of the points near the median depth, which belong to the object
    cv::Point3d min_point(1e9, 1e9, 1e9), max_point(-1e9, -1e9, -1e9);
    for (const cv::Point3d& p : points)
    {
      if (p.z < min_depth_ || p.z > max_depth_ || std::abs(p.z - median) > depth_threshold_)
        continue;
      min_point = cv::Point3d(std::min(min_point.x, p.x), std::min(min_point.y, p.y), std::min(min_point.z, p.z));
      max_point = cv::Point3d(std::max(max_point.x, p.x), std::max(max_point.y, p.y), std::max(max_point.z, p.z));
    }
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

  // publish the 3D boxes of the points of the 2D boxes, and their TF frames
  template <class Stamp>
  void publish(const Stamp& stamp, const std::string& frame_id, const std::vector<Box2D>& input,
               const std::vector<std::vector<cv::Point3d> >& points,
               const std::vector<std::vector<Keypoint3D> >& keypoints)
  {
    vision_msgs::msg::Detection3DArray detections_msg;
    detections_msg.header.stamp = stamp;
    detections_msg.header.frame_id = frame_id;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    // the markers of rviz: delete the markers of the previous image, then a box and a label of each object
    visualization_msgs::msg::MarkerArray markers_msg;
    visualization_msgs::msg::Marker delete_all;
    delete_all.header = detections_msg.header;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers_msg.markers.push_back(delete_all);
    std::vector<bool> matched(tracks_.size(), false);
    for (size_t i = 0; i < input.size(); i++)
    {
      const Box2D& box = input[i];
      cv::Point3d center, size;
      if (!box3D(points[i], center, size))
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

      // a color of each label
      cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(std::hash<std::string>()(box.label) % 180, 255, 255)), bgr;
      cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
      cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
      visualization_msgs::msg::Marker marker;
      marker.header = detections_msg.header;
      marker.ns = "boxes";
      marker.id = static_cast<int>(markers_msg.markers.size());
      // the 12 edges of the box (not filled, not to hide the points and the skeletons in it)
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = detection.bbox.center;
      marker.scale.x = std::max(0.01, center.z * 0.004);  // the width of the lines, wider for the far objects
      for (int e = 0; e < 12; e++)
      {
        // the edges along the axis e / 4, at the corners (-1 or 1) of the other two axes of e % 4
        int axis = e / 4, sign1 = (e % 4) & 1 ? 1 : -1, sign2 = (e % 4) & 2 ? 1 : -1;
        for (int end : { -1, 1 })
        {
          double xyz[3];
          xyz[axis] = end;
          xyz[(axis + 1) % 3] = sign1;
          xyz[(axis + 2) % 3] = sign2;
          geometry_msgs::msg::Point p;
          p.x = xyz[0] * size.x / 2;
          p.y = xyz[1] * size.y / 2;
          p.z = xyz[2] * size.z / 2;
          marker.points.push_back(p);
        }
      }
      marker.color.r = c[2] / 255.0f;
      marker.color.g = c[1] / 255.0f;
      marker.color.b = c[0] / 255.0f;
      marker.color.a = 1.0f;
      markers_msg.markers.push_back(marker);
      visualization_msgs::msg::Marker label = marker;
      label.points.clear();
      label.ns = "labels";
      label.id = static_cast<int>(markers_msg.markers.size());
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose.position.y -= size.y / 2 + 0.1;  // above the box, y of the camera frame is down
      label.scale.x = label.scale.y = 0.0;
      label.scale.z = std::max(0.15, center.z * 0.06);  // the height of the text, larger for the far objects
      label.color.a = 1.0f;
      label.text = cv::format("%s\n%.1fm", name.c_str(), cv::norm(center));
      markers_msg.markers.push_back(label);

      // the axes of the TF frame of the object (x red, y green, z blue), to show the frames of the objects only, not
      // the other frames and the lines between the frames of the TF display of rviz
      double axis_length = std::max(0.15, std::min(size.x, size.y) * 0.5);
      for (int axis = 0; axis < 3; axis++)
      {
        visualization_msgs::msg::Marker arrow;
        arrow.header = detections_msg.header;
        arrow.ns = "frames";
        arrow.id = static_cast<int>(markers_msg.markers.size());
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.orientation.w = 1.0;
        geometry_msgs::msg::Point start, end;
        start.x = end.x = center.x;
        start.y = end.y = center.y;
        start.z = end.z = center.z;
        (axis == 0 ? end.x : axis == 1 ? end.y : end.z) += axis_length;
        arrow.points.push_back(start);
        arrow.points.push_back(end);
        arrow.scale.x = axis_length * 0.1;  // the diameter of the shaft
        arrow.scale.y = axis_length * 0.2;  // the diameter of the head
        arrow.color.r = axis == 0 ? 1.0f : 0.0f;
        arrow.color.g = axis == 1 ? 1.0f : 0.0f;
        arrow.color.b = axis == 2 ? 1.0f : 0.0f;
        arrow.color.a = 1.0f;
        markers_msg.markers.push_back(arrow);
      }

      // the keypoints of a skeleton: the TF frame <name>_<keypoint> of each, and the joints and the bones
      if (keypoints[i].empty())
        continue;
      visualization_msgs::msg::Marker joints = marker;
      joints.ns = "joints";
      joints.id = static_cast<int>(markers_msg.markers.size());
      joints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      joints.pose = geometry_msgs::msg::Pose();
      joints.pose.orientation.w = 1.0;
      joints.scale.x = joints.scale.y = joints.scale.z = box.num_keypoints == 21 ? 0.01 : 0.04;
      joints.color.a = 1.0f;
      std::map<int, cv::Point3d> positions;
      for (const Keypoint3D& keypoint : keypoints[i])
      {
        positions[keypoint.index] = keypoint.position;
        geometry_msgs::msg::Point point;
        point.x = keypoint.position.x;
        point.y = keypoint.position.y;
        point.z = keypoint.position.z;
        joints.points.push_back(point);
        geometry_msgs::msg::TransformStamped joint_transform = transform;
        joint_transform.child_frame_id = tf_prefix_ + name + "_" + keypoint.name;
        joint_transform.transform.translation.x = keypoint.position.x;
        joint_transform.transform.translation.y = keypoint.position.y;
        joint_transform.transform.translation.z = keypoint.position.z;
        transforms.push_back(joint_transform);
      }
      markers_msg.markers.push_back(joints);
      visualization_msgs::msg::Marker bones = joints;
      bones.ns = "bones";
      bones.id = static_cast<int>(markers_msg.markers.size());
      bones.type = visualization_msgs::msg::Marker::LINE_LIST;
      bones.scale.x = joints.scale.x / 2;
      bones.points.clear();
      size_t num_bones = box.num_keypoints == 33 ? sizeof(POSE_BONES) / sizeof(POSE_BONES[0]) :
                         box.num_keypoints == 21 ? sizeof(HAND_BONES) / sizeof(HAND_BONES[0]) :
                                                   0;
      for (size_t b = 0; b < num_bones; b++)
      {
        const int* bone = box.num_keypoints == 33 ? POSE_BONES[b] : HAND_BONES[b];
        if (positions.count(bone[0]) == 0 || positions.count(bone[1]) == 0)
          continue;
        for (int end = 0; end < 2; end++)
        {
          geometry_msgs::msg::Point point;
          point.x = positions[bone[end]].x;
          point.y = positions[bone[end]].y;
          point.z = positions[bone[end]].z;
          bones.points.push_back(point);
        }
      }
      if (!bones.points.empty())
        markers_msg.markers.push_back(bones);
    }
    // forget the tracks lost for track_lost_frames images
    for (size_t i = 0; i < matched.size(); i++)
      if (!matched[i])
        tracks_[i].lost++;
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const Track& track) { return track.lost > track_lost_frames_; }),
                  tracks_.end());

    detections_pub_->publish(detections_msg);
    markers_pub_->publish(markers_msg);
    if (publish_tf_ && !transforms.empty())
      tf_broadcaster_->sendTransform(transforms);
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

  // synchronize the depth source (depth_sub_ or points_sub_), camera_info and the input
  template <class D, class T, class DepthFilter>
  void connect(DepthFilter& depth_filter, message_filters::Subscriber<T>& input_sub,
               void (Detection3DNodelet::*callback)(const typename D::ConstSharedPtr&,
                                                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr&,
                                                    const typename T::ConstSharedPtr&))
  {
    subscribeFilter(input_sub, "input");
    auto sync = std::make_shared<Sync<D, T> >(SyncPolicy<D, T>(queue_size_));
    sync->connectInput(depth_filter, info_sub_, input_sub);
    sync->registerCallback(
        std::bind(callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    sync_ = sync;
  }

  template <class T>
  void connectInput(message_filters::Subscriber<T>& input_sub)
  {
    if (depth_source_ == "points")
      connect<sensor_msgs::msg::PointCloud2, T>(points_sub_, input_sub, &Detection3DNodelet::pointsCallback<T>);
    else
      connect<sensor_msgs::msg::Image, T>(depth_sub_, input_sub, &Detection3DNodelet::depthCallback<T>);
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "subscribe");
    if (depth_source_ == "points")
    {
      subscribeFilter(points_sub_, "points");
    }
    else
    {
      // image_transport takes rclcpp::QoS since lyrical
#if __has_include(<image_transport/node_interfaces.hpp>)
      depth_sub_.subscribe(*this, "depth", "raw", subscriberQos(queue_size_));
#else
      depth_sub_.subscribe(this, "depth", "raw", subscriberQos(queue_size_).get_rmw_qos_profile());
#endif
    }
    subscribeFilter(info_sub_, "camera_info");
    if (input_type_ == "faces")
      connectInput(faces_sub_);
    else if (input_type_ == "rects")
      connectInput(rects_sub_);
    else if (input_type_ == "skeletons")
      connectInput(skeletons_sub_);
    else
      connectInput(detections_sub_);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "unsubscribe");
    sync_.reset();
    depth_sub_.unsubscribe();
    points_sub_.unsubscribe();
    info_sub_.unsubscribe();
    detections_sub_.unsubscribe();
    faces_sub_.unsubscribe();
    rects_sub_.unsubscribe();
    skeletons_sub_.unsubscribe();
  }

public:
  explicit Detection3DNodelet(const rclcpp::NodeOptions& options) : Nodelet("detection_3d", options)
  {
    queue_size_ = declare_parameter("queue_size", 10);
    input_type_ = declare_parameter(
        "input_type", std::string("detections"),
        paramDescriptor("Type of input: detections (opencv_apps/DetectionArrayStamped, e.g. ~/detections of "
                        "dnn_object_detection), faces (FaceArrayStamped, ~/faces of the face nodes), rects "
                        "(RectArrayStamped, ~/output of the tracking nodes) or skeletons (SkeletonArrayStamped, "
                        "~/skeletons of dnn_pose_estimation and dnn_hand_pose_estimation, with the TF frames of the "
                        "keypoints)."));
    if (input_type_ != "detections" && input_type_ != "faces" && input_type_ != "rects" && input_type_ != "skeletons")
    {
      RCLCPP_ERROR(get_logger(), "input_type %s is not detections, faces, rects nor skeletons, use detections",
                   input_type_.c_str());
      input_type_ = "detections";
    }
    depth_source_ = declare_parameter(
        "depth_source", std::string("depth"),
        paramDescriptor("Source of the distances: depth (the depth image registered to the color image, 16UC1 in mm "
                        "or 32FC1 in m, e.g. of an RGB-D camera) or points (a point cloud, e.g. of a LiDAR, "
                        "transformed to the frame of camera_info by TF)."));
    if (depth_source_ != "depth" && depth_source_ != "points")
    {
      RCLCPP_ERROR(get_logger(), "depth_source %s is not depth nor points, use depth", depth_source_.c_str());
      depth_source_ = "depth";
    }
    roi_scale_ = declare_parameter(
        "roi_scale", 0.8,
        paramDescriptor("Scale of the 2D box whose depths are used, smaller to leave out the background at the "
                        "edges of the box.",
                        0.1, 1.0));
    depth_threshold_ = declare_parameter(
        "depth_threshold", 0.3,
        paramDescriptor("The points within this distance (m) of the median depth of the box belong to the object.",
                        0.01, 10.0));
    min_depth_ = declare_parameter("min_depth", 0.1, paramDescriptor("Minimum valid depth (m).", 0.0, 100.0));
    max_depth_ = declare_parameter("max_depth", 10.0, paramDescriptor("Maximum valid depth (m).", 0.0, 1000.0));
    rectified_ = declare_parameter(
        "rectified", false,
        paramDescriptor("The 2D results are of the rectified image (e.g. image_rect), of P and R (the rotation from "
                        "the optical frame) of camera_info, instead of the raw image with the distortion of K and D"));
    publish_tf_ = declare_parameter(
        "publish_tf", true,
        paramDescriptor("Publish the TF frame <tf_prefix><label>_<id> of each object; with it at the start, the node "
                        "subscribes the inputs always, for the subscribers of /tf."));
    tf_prefix_ = declare_parameter("tf_prefix", std::string(""), paramDescriptor("Prefix of the TF frames."));
    track_distance_ = declare_parameter(
        "track_distance", 0.5,
        paramDescriptor("An object within this distance (m) of an object of the same label in the previous images is "
                        "the same object, and keeps its id.",
                        0.0, 10.0));
    track_lost_frames_ = declare_parameter(
        "track_lost_frames", 10, paramDescriptor("Number of images that a lost object keeps its id for.", 0, 1000));
    keypoint_radius_ = declare_parameter(
        "keypoint_radius", 3,
        paramDescriptor("The depth of a keypoint of a skeleton is the median of the depths of the pixels (or the "
                        "points projected) within this distance (pixels) of the keypoint.",
                        0, 50));
    keypoint_visibility_ =
        declare_parameter("keypoint_visibility", 0.5,
                          paramDescriptor("Minimum visibility of a keypoint of a skeleton to be localized.", 0.0, 1.0));
    face_confidence_ = declare_parameter(
        "face_confidence", std::string("probability"),
        paramDescriptor("The confidence of the faces (input_type faces): probability (e.g. dnn_face_detection, "
                        "dnn_face_recognition), the score in [0, 1], or distance (face_recognition, smaller is "
                        "better), whose score is 1 / (1 + distance)"));
    on_set_parameters_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&Detection3DNodelet::parametersCallback, this, std::placeholders::_1));

    // the TF frames are published only when the inputs are subscribed, and a subscriber of /tf (e.g. TF of rviz) does
    // not subscribe the outputs of the node, so the node always subscribes the inputs with publish_tf at the start
    if (publish_tf_)
      always_subscribe_ = true;
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(*this));
    if (depth_source_ == "points")
    {
#ifdef ROS2
      tf_buffer_.reset(new tf2_ros::Buffer(get_clock()));
#else
      tf_buffer_.reset(new tf2_ros::Buffer());
#endif
      tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    }
    detections_pub_ = advertise<vision_msgs::msg::Detection3DArray>("~/detections", 1);
    // the boxes and the labels of the objects for rviz, which shows them without vision_msgs_rviz_plugins
    markers_pub_ = advertise<visualization_msgs::msg::MarkerArray>("~/markers", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::Detection3DNodelet)
