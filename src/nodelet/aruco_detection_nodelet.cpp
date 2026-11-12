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
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// https://github.com/opencv/opencv/blob/4.x/samples/cpp/tutorial_code/objectDetection/detect_markers.cpp
/**
 * Detect ArUco markers with cv::aruco, which is in objdetect since OpenCV 4.7, and was in opencv_contrib.
 * With the camera info (use_camera_info) and marker_length, the poses of the markers are also published.
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#define OPENCV_APPS_ARUCO_DETECTOR
#else
#include <opencv2/aruco.hpp>
#endif

#include <geometry_msgs/msg/pose_array.hpp>
#include <opencv_apps/msg/detection.hpp>
#include <opencv_apps/msg/detection_array_stamped.hpp>

#include <cmath>
#include <map>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
class ArucoDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::DetectionArrayStamped>::SharedPtr msg_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string dictionary_;
  double marker_length_;
  bool rectified_;

  std::mutex mutex_;
  bool need_detector_update_;
#ifdef OPENCV_APPS_ARUCO_DETECTOR
  std::unique_ptr<cv::aruco::ArucoDetector> detector_;
#else
  cv::Ptr<cv::aruco::Dictionary> dictionary_ptr_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
#endif

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "dictionary")
      {
        dictionary_ = parameter.as_string();
        need_detector_update_ = true;
      }
      else if (parameter.get_name() == "marker_length")
        marker_length_ = parameter.as_double();
      else if (parameter.get_name() == "rectified")
        rectified_ = parameter.as_bool();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // predefined dictionary of the name, or -1
  static int dictionaryId(const std::string& name)
  {
    static const std::map<std::string, int> ids = {
      { "DICT_4X4_50", cv::aruco::DICT_4X4_50 },
      { "DICT_4X4_100", cv::aruco::DICT_4X4_100 },
      { "DICT_4X4_250", cv::aruco::DICT_4X4_250 },
      { "DICT_4X4_1000", cv::aruco::DICT_4X4_1000 },
      { "DICT_5X5_50", cv::aruco::DICT_5X5_50 },
      { "DICT_5X5_100", cv::aruco::DICT_5X5_100 },
      { "DICT_5X5_250", cv::aruco::DICT_5X5_250 },
      { "DICT_5X5_1000", cv::aruco::DICT_5X5_1000 },
      { "DICT_6X6_50", cv::aruco::DICT_6X6_50 },
      { "DICT_6X6_100", cv::aruco::DICT_6X6_100 },
      { "DICT_6X6_250", cv::aruco::DICT_6X6_250 },
      { "DICT_6X6_1000", cv::aruco::DICT_6X6_1000 },
      { "DICT_7X7_50", cv::aruco::DICT_7X7_50 },
      { "DICT_7X7_100", cv::aruco::DICT_7X7_100 },
      { "DICT_7X7_250", cv::aruco::DICT_7X7_250 },
      { "DICT_7X7_1000", cv::aruco::DICT_7X7_1000 },
      { "DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL },
      { "DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5 },
      { "DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9 },
      { "DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10 },
      { "DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11 },
    };
    std::map<std::string, int>::const_iterator it = ids.find(name);
    return (it == ids.end()) ? -1 : it->second;
  }

  // (re)create the detector for the dictionary, called with mutex_
  bool updateDetector()
  {
    if (need_detector_update_)
    {
      need_detector_update_ = false;
      int id = dictionaryId(dictionary_);
#ifdef OPENCV_APPS_ARUCO_DETECTOR
      detector_.reset();
      if (id >= 0)
        detector_.reset(new cv::aruco::ArucoDetector(cv::aruco::getPredefinedDictionary(id)));
#else
      dictionary_ptr_.release();
      if (id >= 0)
      {
        dictionary_ptr_ = cv::aruco::getPredefinedDictionary(id);
        detector_parameters_ = cv::aruco::DetectorParameters::create();
      }
#endif
      if (id < 0)
        RCLCPP_ERROR(get_logger(), "Unknown dictionary %s", dictionary_.c_str());
    }
#ifdef OPENCV_APPS_ARUCO_DETECTOR
    return static_cast<bool>(detector_);
#else
    return !dictionary_ptr_.empty();
#endif
  }

  const std::string& frameWithDefault(const std::string& frame, const std::string& image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
  {
    doWork(msg, cam_info);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    doWork(msg, sensor_msgs::msg::CameraInfo::ConstSharedPtr());
  }

  static geometry_msgs::msg::Pose pose(const cv::Vec3d& rvec, const cv::Vec3d& tvec)
  {
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = tvec[0];
    pose_msg.position.y = tvec[1];
    pose_msg.position.z = tvec[2];
    // quaternion of the rotation vector
    double angle = cv::norm(rvec);
    cv::Vec3d axis = (angle > 1e-12) ? rvec / angle : cv::Vec3d(1, 0, 0);
    pose_msg.orientation.x = axis[0] * std::sin(angle / 2);
    pose_msg.orientation.y = axis[1] * std::sin(angle / 2);
    pose_msg.orientation.z = axis[2] * std::sin(angle / 2);
    pose_msg.orientation.w = std::cos(angle / 2);
    return pose_msg;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
              const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      if (frame.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Empty image is given");
        return;
      }

      // Messages
      opencv_apps::msg::DetectionArrayStamped detections_msg;
      detections_msg.header = msg->header;
      geometry_msgs::msg::PoseArray poses_msg;
      poses_msg.header = msg->header;
      if (cam_info)
        poses_msg.header.frame_id = frameWithDefault(cam_info->header.frame_id, msg->header.frame_id);

      // Do the work
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f> > corners;
      double marker_length;
      bool rectified;
      std::string dictionary;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateDetector())
          return;
#ifdef OPENCV_APPS_ARUCO_DETECTOR
        detector_->detectMarkers(frame, corners, ids);
#else
        cv::aruco::detectMarkers(frame, dictionary_ptr_, corners, ids, detector_parameters_);
#endif
        marker_length = marker_length_;
        rectified = rectified_;
        dictionary = dictionary_;
      }
      cv::aruco::drawDetectedMarkers(frame, corners, ids);

      for (size_t i = 0; i < ids.size(); i++)
      {
        opencv_apps::msg::Detection detection_msg;
        detection_msg.label = dictionary;
        detection_msg.id = ids[i];
        detection_msg.score = 1.0;
        cv::Rect2f box = cv::boundingRect(corners[i]);
        detection_msg.rect.x = box.x + box.width / 2.0;
        detection_msg.rect.y = box.y + box.height / 2.0;
        detection_msg.rect.width = box.width;
        detection_msg.rect.height = box.height;
        for (const cv::Point2f& corner : corners[i])
        {
          opencv_apps::msg::Point2D point_msg;
          point_msg.x = corner.x;
          point_msg.y = corner.y;
          detection_msg.corners.push_back(point_msg);
        }
        detections_msg.detections.push_back(detection_msg);
      }

      // poses of the markers, in the order of the detections, only for a calibrated camera (K is all 0 if not)
      cv::Mat camera_matrix, dist_coeffs;
      bool fisheye = false;
      if (cam_info)
      {
// the fields of sensor_msgs/CameraInfo are lower case on ROS2
#ifdef ROS2
        const double* k = cam_info->k.data();
        const double* p = cam_info->p.data();
        const std::vector<double>& d = cam_info->d;
        const std::string& distortion_model = cam_info->distortion_model;
#else
        const double* k = cam_info->K.data();
        const double* p = cam_info->P.data();
        const std::vector<double>& d = cam_info->D;
        const std::string& distortion_model = cam_info->distortion_model;
#endif
        if (rectified)
        {
          // the rectified image (e.g. image_rect of image_proc) has the intrinsics of P, without distortion
          camera_matrix = (cv::Mat_<double>(3, 3) << p[0], p[1], p[2], p[4], p[5], p[6], p[8], p[9], p[10]);
        }
        else
        {
          camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(k)).clone();
          dist_coeffs = cv::Mat(d, true);
          // the 4 coefficients of a fisheye camera, which solvePnP() does not take, the corners are undistorted
          fisheye = distortion_model == "equidistant";
        }
        // K is of the full resolution image, adjust it to the image cropped by roi and binned (0 means 1)
        double binning_x = std::max(1u, static_cast<unsigned int>(cam_info->binning_x));
        double binning_y = std::max(1u, static_cast<unsigned int>(cam_info->binning_y));
        camera_matrix.at<double>(0, 0) /= binning_x;
        camera_matrix.at<double>(0, 2) = (camera_matrix.at<double>(0, 2) - cam_info->roi.x_offset) / binning_x;
        camera_matrix.at<double>(1, 1) /= binning_y;
        camera_matrix.at<double>(1, 2) = (camera_matrix.at<double>(1, 2) - cam_info->roi.y_offset) / binning_y;
        if (camera_matrix.at<double>(0, 0) <= 0 || camera_matrix.at<double>(1, 1) <= 0)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                               "The camera is not calibrated (fx or fy is 0), the poses are not estimated");
          camera_matrix.release();
        }
      }
      if (!camera_matrix.empty() && marker_length > 0 && !ids.empty())
      {
        double l = marker_length / 2;
        // corners of the marker, top-left, top-right, bottom-right and bottom-left, in the marker coordinates
        std::vector<cv::Point3f> object_points = { cv::Point3f(-l, l, 0), cv::Point3f(l, l, 0), cv::Point3f(l, -l, 0),
                                                   cv::Point3f(-l, -l, 0) };
        for (size_t i = 0; i < ids.size(); i++)
        {
          cv::Vec3d rvec, tvec;
          if (fisheye)
          {
            // the corners undistorted by the fisheye model, on the image of camera_matrix without distortion
            std::vector<cv::Point2f> undistorted;
            cv::fisheye::undistortPoints(corners[i], undistorted, camera_matrix, dist_coeffs, cv::noArray(),
                                         camera_matrix);
            cv::solvePnP(object_points, undistorted, camera_matrix, cv::noArray(), rvec, tvec, false,
                         cv::SOLVEPNP_IPPE_SQUARE);
          }
          else
          {
            cv::solvePnP(object_points, corners[i], camera_matrix, dist_coeffs, rvec, tvec, false,
                         cv::SOLVEPNP_IPPE_SQUARE);
          }
          poses_msg.poses.push_back(pose(rvec, tvec));
          if (fisheye)
          {
            // drawFrameAxes() projects with the pinhole model, so project the axes by the fisheye model onto the raw
            // image, x: red, y: green, z: blue as drawFrameAxes()
            std::vector<cv::Point3d> axes = { cv::Point3d(0, 0, 0), cv::Point3d(marker_length, 0, 0),
                                              cv::Point3d(0, marker_length, 0), cv::Point3d(0, 0, marker_length) };
            std::vector<cv::Point2d> projected;
            cv::fisheye::projectPoints(axes, projected, rvec, tvec, camera_matrix, dist_coeffs);
            const cv::Scalar colors[] = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0) };
            for (int j = 0; j < 3; j++)
              cv::line(frame, projected[0], projected[j + 1], colors[j], 3);
          }
          else
          {
            cv::drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, static_cast<float>(marker_length));
          }
        }
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("ArUco detection", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(detections_msg);
      if (!camera_matrix.empty() && marker_length > 0)
        pose_pub_->publish(poses_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &ArucoDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &ArucoDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit ArucoDetectionNodelet(const rclcpp::NodeOptions& options) : Nodelet("aruco_detection", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    need_detector_update_ = true;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to, to get the camera model "
                        "for the poses of the markers. Otherwise the poses are not published."));
    dictionary_ =
        declare_parameter("dictionary", std::string("DICT_4X4_50"),
                          paramDescriptor("Predefined dictionary of the markers, e.g. DICT_4X4_50, DICT_6X6_250, "
                                          "DICT_ARUCO_ORIGINAL, DICT_APRILTAG_36h11"));
    marker_length_ = declare_parameter(
        "marker_length", 0.0,
        paramDescriptor("Length of the side of the markers in meters, to publish their poses with the camera info, "
                        "0 not to publish",
                        0.0, 100.0));
    rectified_ = declare_parameter(
        "rectified", false,
        paramDescriptor("The image is rectified (e.g. image_rect of image_proc), then the poses are estimated with "
                        "the intrinsics of P of the camera info without distortion, instead of K and D"));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&ArucoDetectionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::DetectionArrayStamped>("~/detections", 1);
    pose_pub_ = advertise<geometry_msgs::msg::PoseArray>("~/poses", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::ArucoDetectionNodelet)
