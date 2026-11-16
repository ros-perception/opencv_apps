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


// https://github.com/PRBonn/kiss-icp
/**
 * Odometry of a LiDAR, e.g. Livox (Mid-360, Avia) of the non-repetitive scanning, whose consecutive scans do not see
 * the same points: each scan (sensor_msgs/PointCloud2, or sensor_msgs/LaserScan of a 2D LiDAR) is registered to the
 * local map of the previous scans by ICP, as KISS-ICP: the scan downsampled by voxels, the initial guess of the
 * constant velocity, the closest points in the voxels of the map around each point, and the rigid transform of the
 * robust (Geman-McClure) weights by SVD; the map keeps the points within max_range of the sensor. The motion is
 * published as the odometry of the robot (base_frame) in odom_frame, as rgbd_odometry.
 */

#include "opencv_apps/odometry_nodelet.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace opencv_apps
{
namespace
{
// the key of the voxel of a point
int64_t voxelKey(int x, int y, int z)
{
  const int64_t mask = (1 << 21) - 1;
  return ((static_cast<int64_t>(x) & mask) << 42) | ((static_cast<int64_t>(y) & mask) << 21) |
         (static_cast<int64_t>(z) & mask);
}

cv::Vec3i voxelIndex(const cv::Point3d& p, double size)
{
  return cv::Vec3i(static_cast<int>(std::floor(p.x / size)), static_cast<int>(std::floor(p.y / size)),
                   static_cast<int>(std::floor(p.z / size)));
}

// the points of the first point of each voxel
std::vector<cv::Point3d> voxelDownsample(const std::vector<cv::Point3d>& points, double size)
{
  std::unordered_set<int64_t> voxels;
  std::vector<cv::Point3d> downsampled;
  for (const cv::Point3d& p : points)
  {
    cv::Vec3i v = voxelIndex(p, size);
    if (voxels.insert(voxelKey(v[0], v[1], v[2])).second)
      downsampled.push_back(p);
  }
  return downsampled;
}

cv::Point3d transformPoint(const cv::Matx44d& t, const cv::Point3d& p)
{
  return cv::Point3d(t(0, 0) * p.x + t(0, 1) * p.y + t(0, 2) * p.z + t(0, 3),
                     t(1, 0) * p.x + t(1, 1) * p.y + t(1, 2) * p.z + t(1, 3),
                     t(2, 0) * p.x + t(2, 1) * p.y + t(2, 2) * p.z + t(2, 3));
}

// the local map of the points in the voxels, up to max_points in each voxel
class VoxelMap
{
  std::unordered_map<int64_t, std::vector<cv::Point3d> > voxels_;
  double size_;
  int max_points_;

public:
  VoxelMap(double size, int max_points) : size_(size), max_points_(max_points)
  {
  }

  bool empty() const
  {
    return voxels_.empty();
  }

  void clear()
  {
    voxels_.clear();
  }

  void add(const std::vector<cv::Point3d>& points)
  {
    for (const cv::Point3d& p : points)
    {
      cv::Vec3i v = voxelIndex(p, size_);
      std::vector<cv::Point3d>& voxel = voxels_[voxelKey(v[0], v[1], v[2])];
      if (static_cast<int>(voxel.size()) < max_points_)
        voxel.push_back(p);
    }
  }

  // remove the voxels farther than max_range from the center
  void removeFar(const cv::Point3d& center, double max_range)
  {
    for (auto it = voxels_.begin(); it != voxels_.end();)
    {
      const cv::Point3d d = it->second.front() - center;
      if (d.dot(d) > max_range * max_range)
        it = voxels_.erase(it);
      else
        ++it;
    }
  }

  // the closest point within max_distance of the point, false if there is none: the voxels of the shells around the
  // voxel of the point (max(|dx|, |dy|, |dz|) = k, in the x-y plane if planar), until the closest point is nearer
  // than the next shell, whose points are at least k voxels away; usually in the first shell
  bool closest(const cv::Point3d& p, double max_distance, bool planar, cv::Point3d& closest_point) const
  {
    cv::Vec3i v = voxelIndex(p, size_);
    const int shells = std::max(1, static_cast<int>(std::ceil(max_distance / size_)));
    double best = max_distance * max_distance;
    bool found = false;
    for (int k = 0; k <= shells; k++)
    {
      const int kz = planar ? 0 : k;
      for (int dx = -k; dx <= k; dx++)
        for (int dy = -k; dy <= k; dy++)
          for (int dz = -kz, step = !planar && std::max(std::abs(dx), std::abs(dy)) < k ? 2 * k : 1; dz <= kz;
               dz += step)
          {
            if (std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))) != k)
              continue;  // the voxels of the shell k only
            auto it = voxels_.find(voxelKey(v[0] + dx, v[1] + dy, v[2] + dz));
            if (it == voxels_.end())
              continue;
            for (const cv::Point3d& q : it->second)
            {
              const cv::Point3d d = q - p;
              double distance = d.dot(d);
              if (distance <= best)
              {
                best = distance;
                closest_point = q;
                found = true;
              }
            }
          }
      if (found && best < (k * size_) * (k * size_))
        break;
    }
    return found;
  }
};

// the rigid transform of the source points onto the target points of the weights (the weighted Kabsch algorithm),
// in the x-y plane (rotation about z) if planar
cv::Matx44d rigidTransform(const std::vector<cv::Point3d>& source, const std::vector<cv::Point3d>& target,
                           const std::vector<double>& weights, bool planar)
{
  double total = 0.0;
  cv::Point3d source_center(0, 0, 0), target_center(0, 0, 0);
  for (size_t i = 0; i < source.size(); i++)
  {
    total += weights[i];
    source_center += weights[i] * source[i];
    target_center += weights[i] * target[i];
  }
  cv::Matx44d transform = cv::Matx44d::eye();
  if (total <= 0.0)
    return transform;
  source_center *= 1.0 / total;
  target_center *= 1.0 / total;
  cv::Matx33d covariance = cv::Matx33d::zeros();
  for (size_t i = 0; i < source.size(); i++)
  {
    cv::Point3d s = source[i] - source_center, t = target[i] - target_center;
    covariance += weights[i] * cv::Matx33d(s.x * t.x, s.x * t.y, s.x * t.z, s.y * t.x, s.y * t.y, s.y * t.z, s.z * t.x,
                                           s.z * t.y, s.z * t.z);
  }
  cv::Matx33d rotation;
  if (planar)
  {
    double angle = std::atan2(covariance(0, 1) - covariance(1, 0), covariance(0, 0) + covariance(1, 1));
    rotation = cv::Matx33d(std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle), 0, 0, 0, 1);
  }
  else
  {
    cv::Mat w, u, vt;
    cv::SVD::compute(cv::Mat(covariance), w, u, vt);
    cv::Matx33d v = cv::Matx33d(vt).t(), ut = cv::Matx33d(u).t();
    // a rotation, not a reflection
    cv::Matx33d d = cv::Matx33d::eye();
    d(2, 2) = cv::determinant(v * ut) < 0 ? -1.0 : 1.0;
    rotation = v * d * ut;
  }
  cv::Vec3d translation = cv::Vec3d(target_center) - rotation * cv::Vec3d(source_center);
  if (planar)
    translation[2] = 0.0;
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
      transform(r, c) = rotation(r, c);
    transform(r, 3) = translation[r];
  }
  return transform;
}
}  // namespace

class LaserOdometryNodelet : public opencv_apps::OdometryNodelet
{
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  std::string input_type_;
  double voxel_size_;
  double min_range_;
  double max_range_;
  int max_points_per_voxel_;
  int max_iterations_;
  double correspondence_distance_;

  // the map and the state, set by the callbacks, which may run in other threads (e.g. the multithreaded nodelet
  // manager of ROS 1)
  std::mutex mutex_;
  VoxelMap map_;
  cv::Matx44d pose_;             // the sensor in odom of the sensor (the sensor at the first scan)
  cv::Matx44d previous_motion_;  // for the initial guess of the constant velocity

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "min_range")
        min_range_ = parameter.as_double();
      else if (parameter.get_name() == "max_range")
        max_range_ = parameter.as_double();
      else if (parameter.get_name() == "max_iterations")
        max_iterations_ = parameter.as_int();
      else if (parameter.get_name() == "correspondence_distance")
        correspondence_distance_ = parameter.as_double();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // the points of the cloud (x, y and z of FLOAT32), in the rows of row_step, which may have the padding
  std::vector<cv::Point3d> cloudPoints(const sensor_msgs::msg::PointCloud2& cloud)
  {
    std::vector<cv::Point3d> points;
    size_t offsets[3];
    const char* names[3] = { "x", "y", "z" };
    for (int i = 0; i < 3; i++)
    {
      const char* name = names[i];
      auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                                [name](const sensor_msgs::msg::PointField& f) { return f.name == name; });
      if (field == cloud.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
          field->offset + sizeof(float) > cloud.point_step)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "the field %s of the points is not FLOAT32", name);
        return points;
      }
      offsets[i] = field->offset;
    }
    if (static_cast<size_t>(cloud.width) * cloud.point_step > cloud.row_step ||
        static_cast<size_t>(cloud.height) * cloud.row_step > cloud.data.size())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "the size of the points (%u x %u) is not of their data",
                           cloud.width, cloud.height);
      return points;
    }
    const uint16_t one = 1;
    const bool host_big_endian = *reinterpret_cast<const uint8_t*>(&one) == 0;
    const bool swap_bytes = static_cast<bool>(cloud.is_bigendian) != host_big_endian;
    for (size_t row = 0; row < cloud.height; row++)
      for (size_t column = 0; column < cloud.width; column++)
      {
        const uint8_t* point = cloud.data.data() + row * cloud.row_step + column * cloud.point_step;
        float xyz[3];
        for (int i = 0; i < 3; i++)
        {
          // the bytes of the float in the byte order of this host
          uint8_t bytes[sizeof(float)];
          std::memcpy(bytes, point + offsets[i], sizeof(float));
          if (swap_bytes)
            std::reverse(bytes, bytes + sizeof(float));
          std::memcpy(&xyz[i], bytes, sizeof(float));
        }
        if (std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]))
          points.push_back(cv::Point3d(xyz[0], xyz[1], xyz[2]));
      }
    return points;
  }

  static std::vector<cv::Point3d> scanPoints(const sensor_msgs::msg::LaserScan& scan)
  {
    std::vector<cv::Point3d> points;
    for (size_t i = 0; i < scan.ranges.size(); i++)
    {
      float range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max)
        continue;
      double angle = scan.angle_min + i * scan.angle_increment;
      points.push_back(cv::Point3d(range * std::cos(angle), range * std::sin(angle), 0.0));
    }
    return points;
  }

  // register the points (in the sensor frame) to the map from the initial guess of the pose of the sensor
  cv::Matx44d registerPoints(const std::vector<cv::Point3d>& points, const cv::Matx44d& guess, bool planar) const
  {
    cv::Matx44d pose = guess;
    // the kernel of Geman-McClure of a third of the correspondence distance, as KISS-ICP
    const double kernel = correspondence_distance_ / 3.0;
    for (int iteration = 0; iteration < max_iterations_; iteration++)
    {
      std::vector<cv::Point3d> source, target;
      std::vector<double> weights;
      for (const cv::Point3d& p : points)
      {
        cv::Point3d q = transformPoint(pose, p), closest;
        if (!map_.closest(q, correspondence_distance_, planar, closest))
          continue;
        cv::Point3d d = closest - q;
        double residual2 = d.dot(d);
        source.push_back(q);
        target.push_back(closest);
        double w = kernel / (kernel + residual2);
        weights.push_back(w * w);
      }
      if (source.size() < 10)
        break;
      cv::Matx44d update = rigidTransform(source, target, weights, planar);
      pose = update * pose;
      // converged when both the translation and the rotation of the update are small
      cv::Vec3d axis_angle;
      cv::Rodrigues(cv::Matx33d(update.get_minor<3, 3>(0, 0)), axis_angle);
      if (cv::norm(cv::Vec3d(update(0, 3), update(1, 3), update(2, 3))) < 1e-4 && cv::norm(axis_angle) < 1e-4)
        break;
    }
    return pose;
  }

  void process(const std_msgs::msg::Header& header, const std::vector<cv::Point3d>& raw_points, bool planar)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try
    {
      if (!sensorOnBase(header.frame_id))
        return;
      std::vector<cv::Point3d> points;
      for (const cv::Point3d& p : raw_points)
      {
        double range = cv::norm(p);
        if (range >= min_range_ && range <= max_range_)
          points.push_back(p);
      }
      if (points.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "no point within min_range and max_range");
        return;
      }
      // restart the odometry at the first scan, or when the time goes back, e.g. a bag played in a loop
      if (restartOdometry(rclcpp::Time(header.stamp)))
      {
        map_ = VoxelMap(voxel_size_, max_points_per_voxel_);
        pose_ = cv::Matx44d::eye();
        previous_motion_ = cv::Matx44d::eye();
      }
      // the points to register (half of the voxel of the map), and to add to the map
      std::vector<cv::Point3d> frame = voxelDownsample(points, voxel_size_ * 0.5);
      std::vector<cv::Point3d> source = voxelDownsample(frame, voxel_size_ * 1.5);
      cv::Matx44d motion = cv::Matx44d::eye();
      if (!map_.empty())
      {
        cv::Matx44d pose = registerPoints(source, pose_ * previous_motion_, planar);
        motion = pose_.inv() * pose;
        pose_ = pose;
      }
      previous_motion_ = motion;
      std::vector<cv::Point3d> frame_in_odom;
      for (const cv::Point3d& p : frame)
        frame_in_odom.push_back(transformPoint(pose_, p));
      map_.add(frame_in_odom);
      map_.removeFar(cv::Point3d(pose_(0, 3), pose_(1, 3), pose_(2, 3)), max_range_);
      publishOdometry(header, motion);
      addToMap(header, frame, std::vector<cv::Vec3b>());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
  }

  void pointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    process(msg->header, cloudPoints(*msg), false);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
  {
    process(msg->header, scanPoints(*msg), true);
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "subscribe");
    if (input_type_ == "scan")
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", subscriberQos(queue_size_),
                                                                   std::bind(&LaserOdometryNodelet::scanCallback, this,
                                                                             std::placeholders::_1));
    else
      points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("points", subscriberQos(queue_size_),
                                                                       std::bind(&LaserOdometryNodelet::pointsCallback,
                                                                                 this, std::placeholders::_1));
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "unsubscribe");
    points_sub_.reset();
    scan_sub_.reset();
  }

public:
  explicit LaserOdometryNodelet(const rclcpp::NodeOptions& options)
    : OdometryNodelet("laser_odometry", options)
    , map_(0.5, 20)
    , pose_(cv::Matx44d::eye())
    , previous_motion_(cv::Matx44d::eye())
  {
    queue_size_ = declare_parameter("queue_size", 10);
    input_type_ = declare_parameter(
        "input_type", std::string("points"),
        paramDescriptor("Type of input: points (sensor_msgs/PointCloud2 of a 3D LiDAR, e.g. Livox) or scan "
                        "(sensor_msgs/LaserScan of a 2D LiDAR, whose motion is in its x-y plane)"));
    if (input_type_ != "points" && input_type_ != "scan")
    {
      RCLCPP_ERROR(get_logger(), "input_type %s is not points nor scan, use points", input_type_.c_str());
      input_type_ = "points";
    }
    voxel_size_ = declare_parameter(
        "voxel_size", 0.5,
        paramDescriptor("Size (m) of the voxels of the map, e.g. 0.5 outdoors and 0.1 - 0.2 indoors; the scan is "
                        "downsampled by its half to the map and by 1.5 times of it to the registration.",
                        0.01, 10.0));
    max_points_per_voxel_ = declare_parameter("max_points_per_voxel", 20,
                                              paramDescriptor("Maximum points in a voxel of the map.", 1, 1000));
    min_range_ = declare_parameter("min_range", 0.3,
                                   paramDescriptor("Minimum range (m) of the points, without the robot.", 0.0, 100.0));
    max_range_ = declare_parameter("max_range", 50.0,
                                   paramDescriptor("Maximum range (m) of the points, and of the map.", 0.1, 1000.0));
    max_iterations_ =
        declare_parameter("max_iterations", 50, paramDescriptor("Maximum iterations of ICP of each scan.", 1, 1000));
    correspondence_distance_ = declare_parameter(
        "correspondence_distance", 1.0,
        paramDescriptor("Maximum distance (m) of the corresponding points of the scan and the map, larger than the "
                        "error of the initial guess of the constant velocity.",
                        0.01, 10.0));
    map_ = VoxelMap(voxel_size_, max_points_per_voxel_);
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&LaserOdometryNodelet::parametersCallback, this, std::placeholders::_1));

    initOdometry();

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::LaserOdometryNodelet)
