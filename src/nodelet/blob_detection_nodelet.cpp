// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Hui Shi, University of Tartu.
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
 *   * Neither the name of the Hui Shi, University of Tartu nor the names of its
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

// https://github.com/opencv/opencv/blob/4.x/samples/cpp/detect_blob.cpp
// https://github.com/spmallick/learnopencv/blob/master/BlobDetector/blob.cpp
/**
 * Detect blobs with cv::SimpleBlobDetector. The input is typically a binary mask, e.g. the output of
 * hsv_color_filter or morphology, any image is converted to mono8.
 */

#include <cfloat>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_apps/nodelet.h"
#include "opencv_apps/msg/circle_array_stamped.hpp"

namespace opencv_apps
{
class BlobDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::CircleArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;

  std::mutex mutex_;
  bool use_camera_info_;

  cv::SimpleBlobDetector::Params params_;
  cv::Ptr<cv::SimpleBlobDetector> detector_;

  // circularity, inertia ratio and convexity are at most 1.0, but OpenCV excludes the maximum value
  // (e.g. a convex blob has convexity 1.0), so treat 1.0 as no upper limit
  static float upperLimit(double value)
  {
    return value >= 1.0 ? FLT_MAX : static_cast<float>(value);
  }

  static cv::Ptr<cv::SimpleBlobDetector> createDetector(const cv::SimpleBlobDetector::Params& params)
  {
#if CV_MAJOR_VERSION >= 3
    return cv::SimpleBlobDetector::create(params);
#else
    return new cv::SimpleBlobDetector(params);
#endif
  }

  // KeyPoint::size is the diameter of the blob since OpenCV 3, but the radius on OpenCV 2
  static double blobRadius(const cv::KeyPoint& keypoint)
  {
#if CV_MAJOR_VERSION >= 3
    return keypoint.size / 2.0;
#else
    return keypoint.size;
#endif
  }

  // create the detector and run it on an empty image, as OpenCV rejects some parameters
  // (e.g. min_area larger than max_area) only when detecting
  static cv::Ptr<cv::SimpleBlobDetector> checkedDetector(const cv::SimpleBlobDetector::Params& params,
                                                         std::string& reason)
  {
    if (params.minThreshold >= params.maxThreshold)
    {
      reason = "min_threshold must be smaller than max_threshold";
      return cv::Ptr<cv::SimpleBlobDetector>();
    }
    if (params.filterByArea && params.minArea >= params.maxArea)
    {
      reason = "min_area must be smaller than max_area";
      return cv::Ptr<cv::SimpleBlobDetector>();
    }
    try
    {
      cv::Ptr<cv::SimpleBlobDetector> detector = createDetector(params);
      std::vector<cv::KeyPoint> keypoints;
      detector->detect(cv::Mat::zeros(16, 16, CV_8UC1), keypoints);
      return detector;
    }
    catch (cv::Exception& e)
    {
      reason = e.what();
      return cv::Ptr<cv::SimpleBlobDetector>();
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // apply the parameters to copies so that invalid ones keep the working detector
    bool use_camera_info = use_camera_info_;
    cv::SimpleBlobDetector::Params params = params_;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      const std::string& name = parameter.get_name();
      if (name == "use_camera_info")
        use_camera_info = parameter.as_bool();
      else if (name == "threshold_step")
        params.thresholdStep = static_cast<float>(parameter.as_double());
      else if (name == "min_threshold")
        params.minThreshold = static_cast<float>(parameter.as_double());
      else if (name == "max_threshold")
        params.maxThreshold = static_cast<float>(parameter.as_double());
      else if (name == "min_repeatability")
        params.minRepeatability = static_cast<size_t>(parameter.as_int());
      else if (name == "min_dist_between_blobs")
        params.minDistBetweenBlobs = static_cast<float>(parameter.as_double());
      else if (name == "filter_by_color")
        params.filterByColor = parameter.as_bool();
      else if (name == "blob_color")
        params.blobColor = static_cast<uchar>(parameter.as_int());
      else if (name == "filter_by_area")
        params.filterByArea = parameter.as_bool();
      else if (name == "min_area")
        params.minArea = static_cast<float>(parameter.as_double());
      else if (name == "max_area")
        params.maxArea = static_cast<float>(parameter.as_double());
      else if (name == "filter_by_circularity")
        params.filterByCircularity = parameter.as_bool();
      else if (name == "min_circularity")
        params.minCircularity = static_cast<float>(parameter.as_double());
      else if (name == "max_circularity")
        params.maxCircularity = upperLimit(parameter.as_double());
      else if (name == "filter_by_inertia")
        params.filterByInertia = parameter.as_bool();
      else if (name == "min_inertia_ratio")
        params.minInertiaRatio = static_cast<float>(parameter.as_double());
      else if (name == "max_inertia_ratio")
        params.maxInertiaRatio = upperLimit(parameter.as_double());
      else if (name == "filter_by_convexity")
        params.filterByConvexity = parameter.as_bool();
      else if (name == "min_convexity")
        params.minConvexity = static_cast<float>(parameter.as_double());
      else if (name == "max_convexity")
        params.maxConvexity = upperLimit(parameter.as_double());
    }
    rcl_interfaces::msg::SetParametersResult result;
    cv::Ptr<cv::SimpleBlobDetector> detector = checkedDetector(params, result.reason);
    if (detector.empty())
    {
      result.successful = false;
      return result;
    }
    use_camera_info_ = use_camera_info;
    params_ = params;
    detector_ = detector;
    result.successful = true;
    return result;
  }

  void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    try
    {
      cv::Mat gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;

      std::vector<cv::KeyPoint> keypoints;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (detector_.empty())
        {
          detector_ = createDetector(params_);
        }
        detector_->detect(gray, keypoints);
      }

      opencv_apps::msg::CircleArrayStamped circles_msg;
      circles_msg.header = msg->header;
      circles_msg.header.frame_id = input_frame_from_msg;

      for (const cv::KeyPoint& keypoint : keypoints)
      {
        opencv_apps::msg::Circle circle_msg;
        circle_msg.center.x = keypoint.pt.x;
        circle_msg.center.y = keypoint.pt.y;
        circle_msg.radius = blobRadius(keypoint);
        circles_msg.circles.push_back(circle_msg);
      }
      msg_pub_->publish(circles_msg);

      // draw the blobs only when someone sees them
      if (debug_view_ || img_pub_.getNumSubscribers() > 0)
      {
        cv::Mat out_image;
        cv::cvtColor(gray, out_image, cv::COLOR_GRAY2BGR);
        for (const cv::KeyPoint& keypoint : keypoints)
        {
          cv::Point center(cvRound(keypoint.pt.x), cvRound(keypoint.pt.y));
          cv::circle(out_image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
          cv::circle(out_image, center, cvRound(blobRadius(keypoint)), cv::Scalar(0, 0, 255), 2, 8, 0);
        }

        if (debug_view_)
        {
          cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
          cv::imshow(window_name_, out_image);
          cv::waitKey(1);
        }

        img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, out_image).toImageMsg());
      }
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
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &BlobDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &BlobDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit BlobDetectionNodelet(const rclcpp::NodeOptions& options) : Nodelet("blob_detection", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    window_name_ = "Blob Detection Demo";

    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    params_.thresholdStep =
        declare_parameter("threshold_step", 10.0,
                          paramDescriptor("Step of the thresholds from min_threshold to max_threshold", 1.0, 255.0));
    params_.minThreshold = declare_parameter(
        "min_threshold", 50.0, paramDescriptor("The minimum threshold to binarize the image", 0.0, 255.0));
    params_.maxThreshold = declare_parameter(
        "max_threshold", 220.0, paramDescriptor("The maximum threshold (exclusive) to binarize the image", 0.0, 255.0));
    params_.minRepeatability = declare_parameter(
        "min_repeatability", 2,
        paramDescriptor("The minimum number of the binarized images in which a blob is detected", 1, 255));
    params_.minDistBetweenBlobs = declare_parameter(
        "min_dist_between_blobs", 10.0,
        paramDescriptor("The blobs located closer than min_dist_between_blobs are merged", 0.0, 1000.0));
    params_.filterByColor =
        declare_parameter("filter_by_color", true, paramDescriptor("Filter the blobs by the color of the center"));
    params_.blobColor = declare_parameter(
        "blob_color", 255, paramDescriptor("0 to extract dark blobs and 255 to extract light blobs", 0, 255));
    params_.filterByArea = declare_parameter("filter_by_area", true, paramDescriptor("Filter the blobs by the area"));
    params_.minArea = declare_parameter(
        "min_area", 25.0, paramDescriptor("The minimum area (inclusive) of the blobs [pixel]", 0.0, 1000000.0));
    params_.maxArea = declare_parameter(
        "max_area", 100000.0, paramDescriptor("The maximum area (exclusive) of the blobs [pixel]", 0.0, 1000000.0));
    params_.filterByCircularity = declare_parameter(
        "filter_by_circularity", false, paramDescriptor("Filter the blobs by the circularity (4*pi*area/perimeter^2)"));
    params_.minCircularity = declare_parameter(
        "min_circularity", 0.8, paramDescriptor("The minimum circularity (inclusive) of the blobs", 0.0, 1.0));
    params_.maxCircularity = upperLimit(declare_parameter(
        "max_circularity", 1.0,
        paramDescriptor("The maximum circularity (exclusive) of the blobs, 1.0 means no limit", 0.0, 1.0)));
    params_.filterByInertia = declare_parameter(
        "filter_by_inertia", false,
        paramDescriptor("Filter the blobs by the inertia ratio (minor axis / major axis, 0: line, 1: circle)"));
    params_.minInertiaRatio = declare_parameter(
        "min_inertia_ratio", 0.1, paramDescriptor("The minimum inertia ratio (inclusive) of the blobs", 0.0, 1.0));
    params_.maxInertiaRatio = upperLimit(declare_parameter(
        "max_inertia_ratio", 1.0,
        paramDescriptor("The maximum inertia ratio (exclusive) of the blobs, 1.0 means no limit", 0.0, 1.0)));
    params_.filterByConvexity =
        declare_parameter("filter_by_convexity", false,
                          paramDescriptor("Filter the blobs by the convexity (area / area of convex hull)"));
    params_.minConvexity = declare_parameter(
        "min_convexity", 0.95, paramDescriptor("The minimum convexity (inclusive) of the blobs", 0.0, 1.0));
    params_.maxConvexity = upperLimit(declare_parameter(
        "max_convexity", 1.0,
        paramDescriptor("The maximum convexity (exclusive) of the blobs, 1.0 means no limit", 0.0, 1.0)));
    std::string reason;
    detector_ = checkedDetector(params_, reason);
    if (detector_.empty())
    {
      RCLCPP_ERROR(get_logger(), "Invalid blob detector parameters: %s", reason.c_str());
    }
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&BlobDetectionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::CircleArrayStamped>("~/circles", 1);
    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::BlobDetectionNodelet)
