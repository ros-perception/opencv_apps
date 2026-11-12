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

// https://github.com/opencv/opencv/blob/4.x/samples/cpp/barcode.cpp
/**
 * Detect and decode barcodes (EAN-8, EAN-13, UPC-A, UPC-E) with cv::barcode::BarcodeDetector, which is in objdetect
 * since OpenCV 4.8, and was in opencv_contrib
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 8)
#include <opencv2/objdetect/barcode.hpp>
#else
#include <opencv2/barcode.hpp>
#include <sstream>
#endif

#include <opencv_apps/msg/detection.hpp>
#include <opencv_apps/msg/detection_array_stamped.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
class BarcodeDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::DetectionArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::mutex mutex_;
  std::unique_ptr<cv::barcode::BarcodeDetector> detector_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // detection of the corners, the rect is the bounding box of the corners
  static opencv_apps::msg::Detection detection(const std::string& label, const std::string& data, int id,
                                               const std::vector<cv::Point2f>& corners)
  {
    opencv_apps::msg::Detection detection_msg;
    detection_msg.label = label;
    detection_msg.data = data;
    detection_msg.id = id;
    detection_msg.score = 1.0;
    cv::Rect2f box = cv::boundingRect(corners);
    detection_msg.rect.x = box.x + box.width / 2.0;
    detection_msg.rect.y = box.y + box.height / 2.0;
    detection_msg.rect.width = box.width;
    detection_msg.rect.height = box.height;
    for (const cv::Point2f& corner : corners)
    {
      opencv_apps::msg::Point2D point_msg;
      point_msg.x = corner.x;
      point_msg.y = corner.y;
      detection_msg.corners.push_back(point_msg);
    }
    return detection_msg;
  }

  static void drawCode(cv::Mat& frame, const std::vector<cv::Point2f>& corners, const std::string& text)
  {
    std::vector<cv::Point> polygon(corners.begin(), corners.end());
    cv::polylines(frame, polygon, true, cv::Scalar(0, 255, 0), 2);
    if (!corners.empty())
      cv::circle(frame, polygon[0], 4, cv::Scalar(0, 0, 255), -1);
    cv::Rect box = cv::boundingRect(polygon);
    cv::putText(frame, text, cv::Point(box.x, std::max(box.y - 4, 12)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 0), 3);
    cv::putText(frame, text, cv::Point(box.x, std::max(box.y - 4, 12)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1);
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
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }
  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
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

      // Do the work
      std::vector<std::string> decoded_info;
      std::vector<std::string> decoded_type;
      std::vector<cv::Point2f> points;  // 4 corners of each code
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // false is returned also when the codes are detected but none is decoded, then decoded_info has the empty
        // strings and points has their corners, so the return value is not used
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 8)
        detector_->detectAndDecodeWithType(frame, decoded_info, decoded_type, points);
#else
        std::vector<cv::barcode::BarcodeType> types;
        detector_->detectAndDecode(frame, decoded_info, types, points);
        for (const cv::barcode::BarcodeType& type : types)
        {
          std::stringstream ss;
          ss << type;
          decoded_type.push_back(ss.str());
        }
#endif
      }
      for (size_t i = 0; i < decoded_info.size() && (i + 1) * 4 <= points.size(); i++)
      {
        std::vector<cv::Point2f> corners(points.begin() + i * 4, points.begin() + (i + 1) * 4);
        // the codes detected but not decoded have the empty data, and the type NONE
        std::string type = (i < decoded_type.size()) ? decoded_type[i] : std::string();
        detections_msg.detections.push_back(detection(type, decoded_info[i], -1, corners));
        drawCode(frame, corners, decoded_info[i].empty() ? type : type + " " + decoded_info[i]);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Barcode detection", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(detections_msg);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &BarcodeDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &BarcodeDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit BarcodeDetectionNodelet(const rclcpp::NodeOptions& options) : Nodelet("barcode_detection", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    detector_.reset(new cv::barcode::BarcodeDetector());
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&BarcodeDetectionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::DetectionArrayStamped>("~/detections", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::BarcodeDetectionNodelet)
