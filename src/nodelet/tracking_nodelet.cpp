// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Kei Okada
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

// https://docs.opencv.org/4.x/d2/d0a/tutorial_introduction_to_tracker.html
/**
 * This is a demo of the OpenCV trackers. Select the target with mouse, or call ~set_roi service.
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// This node uses the tracking API of OpenCV 4.5.1 or later (https://github.com/opencv/opencv_contrib/pull/2737):
// cv::Tracker and cv::TrackerMIL are in the video module, and KCF, CSRT and the legacy
// trackers (cv::legacy::TrackerBoosting, ...) are in the tracking module of opencv_contrib.
#include <opencv2/video/tracking.hpp>
#ifdef OPENCV_APPS_HAVE_TRACKING  // set by CMakeLists.txt if opencv_contrib tracking module is found
#include <opencv2/tracking.hpp>
#if __has_include(<opencv2/tracking/tracking_legacy.hpp>)
#include <opencv2/tracking/tracking_legacy.hpp>
#define OPENCV_APPS_HAVE_TRACKING_LEGACY
#endif
#endif

#include <opencv_apps/msg/rect_array_stamped.hpp>
#include <opencv_apps/srv/set_roi.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
// constants of cfg/Tracking.cfg
enum
{
  Tracking_MIL = 0,
  Tracking_BOOSTING = 1,
  Tracking_MEDIANFLOW = 2,
  Tracking_TLD = 3,
  Tracking_KCF = 4,
  Tracking_MOSSE = 6,
  Tracking_CSRT = 7
};

class TrackingNodelet : public opencv_apps::Nodelet
{
  ////////////////////////////////////////////////////////
  // Dynamic Reconfigure
  ////////////////////////////////////////////////////////
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  bool window_created_;  // the window has the mouse callback to this node

  std::string window_name_;

  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::RectArrayStamped>::SharedPtr msg_pub_;
  rclcpp::Service<opencv_apps::srv::SetROI>::SharedPtr roi_srv_;

  std::mutex mutex_;
  bool use_camera_info_;
  int tracking_algorithm_;

  // the tracker is null when no target is set
  cv::Ptr<cv::Tracker> tracker_;
  // the target, which is tracked on the input images
  cv::Rect roi_;
  // the new tracker and target to be initialized on the next input image, which replace tracker_ and roi_ only when
  // the initialization succeeds, e.g. set_roi without image or a new tracking_algorithm
  cv::Ptr<cv::Tracker> pending_tracker_;
  cv::Rect pending_roi_;

  // selection with mouse, as opencv/modules/highgui/src/roiSelector.cpp
  bool selecting_;
  cv::Point selection_start_;
  cv::Rect selection_;
  cv::Size image_size_;

  // mouse events of the window, queued by onMouse() and processed in doWork() of this node, as cv::waitKey() of
  // another node in the same process may call onMouse() of this node
  std::mutex mouse_mutex_;
  std::vector<cv::Vec3i> mouse_events_;  // event, x, y

  static void onMouse(int event, int x, int y, int flags, void* param)
  {
    TrackingNodelet* node = static_cast<TrackingNodelet*>(param);
    std::lock_guard<std::mutex> lock(node->mouse_mutex_);
    node->mouse_events_.push_back(cv::Vec3i(event, x, y));
  }

  // process the queued mouse events, with mutex_ locked
  void processMouseEvents()
  {
    std::vector<cv::Vec3i> events;
    {
      std::lock_guard<std::mutex> lock(mouse_mutex_);
      events.swap(mouse_events_);
    }
    for (const cv::Vec3i& e : events)
      mouseCallback(e[0], e[1], e[2]);
  }

  // called in processMouseEvents(), with mutex_ locked
  void mouseCallback(int event, int x, int y)
  {
    // limit x and y to image size
    x = std::min(std::max(x, 0), image_size_.width);
    y = std::min(std::max(y, 0), image_size_.height);
    switch (event)
    {
      // update the selected bounding box
      case cv::EVENT_MOUSEMOVE:
        if (selecting_)
        {
          selection_ = cv::Rect(selection_start_, cv::Point(x, y));
        }
        break;
      // start to select the bounding box
      case cv::EVENT_LBUTTONDOWN:
        selecting_ = true;
        selection_start_ = cv::Point(x, y);
        selection_ = cv::Rect(x, y, 0, 0);
        break;
      // track the selected bounding box
      case cv::EVENT_LBUTTONUP:
        selecting_ = false;
        selection_ = cv::Rect(selection_start_, cv::Point(x, y));
        if (!selection_.empty())
        {
          std::string error;
          setTarget(selection_, cv::Mat(), error);
        }
        break;
    }
  }

  cv::Ptr<cv::Tracker> createTracker(int tracking_algorithm, std::string& error)
  {
    try
    {
      switch (tracking_algorithm)
      {
        case Tracking_MIL:
          RCLCPP_INFO(get_logger(), "Create MIL (Multiple Instance Learning) tracker");
          return cv::TrackerMIL::create();
#ifdef OPENCV_APPS_HAVE_TRACKING_LEGACY
        case Tracking_BOOSTING:
          RCLCPP_INFO(get_logger(), "Create On-line version of the AdaBoost tracker");
          return cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerBoosting::create());
        case Tracking_MEDIANFLOW:
          RCLCPP_INFO(get_logger(), "Create Median Flow tracker");
          return cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerMedianFlow::create());
        case Tracking_TLD:
          RCLCPP_INFO(get_logger(), "Create TLD (Tracking, learning and detection) tracker");
          return cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerTLD::create());
        case Tracking_MOSSE:
          RCLCPP_INFO(get_logger(), "Create MOSSE (Minimum Output Sum of Squared Error) tracker");
          return cv::legacy::upgradeTrackingAPI(cv::legacy::TrackerMOSSE::create());
#endif
#ifdef OPENCV_APPS_HAVE_TRACKING
        case Tracking_KCF:
          RCLCPP_INFO(get_logger(), "Create KCF (Kernelized Correlation Filter) tracker");
          return cv::TrackerKCF::create();
        case Tracking_CSRT:
          RCLCPP_INFO(get_logger(), "Create CSRT (Discriminative Correlation Filter with Channel and Spatial "
                                    "Reliability) tracker");
          return cv::TrackerCSRT::create();
#endif
        default:
          error = "tracking_algorithm " + std::to_string(tracking_algorithm) +
                  " is not available in this build (OpenCV " + CV_VERSION +
#ifndef OPENCV_APPS_HAVE_TRACKING
                  ", without opencv_contrib tracking module" +
#endif
                  "): KCF and CSRT need opencv_contrib tracking module, BOOSTING, MEDIANFLOW, TLD and MOSSE need "
                  "its legacy API (opencv2/tracking/tracking_legacy.hpp)";
          return cv::Ptr<cv::Tracker>();
      }
    }
    catch (cv::Exception& e)
    {
      error = "Failed to create tracker: " + e.msg;
      return cv::Ptr<cv::Tracker>();
    }
  }

  // set the target to track, on image if it is not empty, or on the next input image. mutex_ must be locked.
  // The current target is kept if the new one fails, e.g. with the ROI out of the image.
  bool setTarget(const cv::Rect& roi, const cv::Mat& image, std::string& error)
  {
    cv::Ptr<cv::Tracker> tracker = createTracker(tracking_algorithm_, error);
    cv::Rect target = roi;
    if (!tracker || (!image.empty() && !initTracker(tracker, target, image, error)))
    {
      RCLCPP_ERROR_STREAM(get_logger(), error);
      return false;
    }
    if (image.empty())
    {
      pending_tracker_ = tracker;
      pending_roi_ = target;
    }
    else
    {
      tracker_ = tracker;
      roi_ = target;
      pending_tracker_.reset();
    }
    return true;
  }

  // initialize pending_tracker_ with pending_roi_ on image, and replace tracker_ with it if it succeeds.
  // mutex_ must be locked.
  bool initPendingTracker(const cv::Mat& image, std::string& error)
  {
    cv::Ptr<cv::Tracker> tracker = pending_tracker_;
    cv::Rect roi = pending_roi_;
    pending_tracker_.reset();
    if (!initTracker(tracker, roi, image, error))
      return false;
    tracker_ = tracker;
    roi_ = roi;
    return true;
  }

  // initialize tracker with roi, which is kept within image
  bool initTracker(const cv::Ptr<cv::Tracker>& tracker, cv::Rect& roi, const cv::Mat& image, std::string& error)
  {
    roi &= cv::Rect(cv::Point(0, 0), image.size());
    if (roi.empty())
    {
      error = "ROI is out of the image";
      return false;
    }
    try
    {
      tracker->init(image, roi);
    }
    catch (cv::Exception& e)
    {
      error = "Failed to initialize tracker: " + e.msg;
      return false;
    }
    RCLCPP_INFO(get_logger(), "Track ROI (center-x: %d, center-y: %d, width: %d, height: %d)", roi.x + roi.width / 2,
                roi.y + roi.height / 2, roi.width, roi.height);
    return true;
  }

  void setROICb(const std::shared_ptr<opencv_apps::srv::SetROI::Request> req,
                std::shared_ptr<opencv_apps::srv::SetROI::Response> res)
  {
    // x-y of opencv_apps/Rect is the center
    cv::Rect roi(cvRound(req->rect.x - req->rect.width / 2), cvRound(req->rect.y - req->rect.height / 2),
                 cvRound(req->rect.width), cvRound(req->rect.height));
    if (roi.empty())
    {
      res->ok = false;
      res->error = "ROI is empty";
      RCLCPP_ERROR_STREAM(get_logger(), res->error);
      return;
    }
    cv::Mat image;
    try
    {
      if (!req->image.data.empty())
      {
        image = cv_bridge::toCvCopy(req->image, sensor_msgs::image_encodings::BGR8)->image;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      res->ok = setTarget(roi, image, res->error);
    }
    catch (std::exception& e)  // cv_bridge::Exception, cv::Exception, ...
    {
      res->ok = false;
      res->error = std::string("Failed to set ROI: ") + e.what();
      RCLCPP_ERROR_STREAM(get_logger(), res->error);
    }
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

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &TrackingNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &TrackingNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    int tracking_algorithm = tracking_algorithm_;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "tracking_algorithm")
        tracking_algorithm = parameter.as_int();
    }
    if (tracking_algorithm != tracking_algorithm_)
    {
      // reject the tracking algorithm which is not available, and keep the current tracker
      cv::Ptr<cv::Tracker> tracker = createTracker(tracking_algorithm, result.reason);
      if (!tracker)
      {
        RCLCPP_ERROR_STREAM(get_logger(), result.reason);
        result.successful = false;
        return result;
      }
      tracking_algorithm_ = tracking_algorithm;
      // track the current (or pending) target with the new tracker, from the next input image
      if (pending_tracker_)
      {
        pending_tracker_ = tracker;
      }
      else if (tracker_)
      {
        pending_tracker_ = tracker;
        pending_roi_ = roi_;
      }
    }
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
    }
    return result;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const std::string& input_frame_from_msg)
  {
    try
    {
      cv::Mat frame = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      if (frame.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Empty image is given");
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);

      bool tracked = false;
      bool initialized = false;
      if (pending_tracker_)
      {
        // the current tracker (if any) keeps tracking when the new one fails
        std::string error;
        initialized = tracked = initPendingTracker(frame, error);
        if (!initialized)
          RCLCPP_ERROR_STREAM(get_logger(), error);
      }
      if (tracker_ && !initialized)
      {
        /// Tracking
        cv::Rect roi;
        tracked = tracker_->update(frame, roi);
        // keep roi within image, the box may be empty (e.g. out of the image) even if update() succeeds
        roi &= cv::Rect(cv::Point(0, 0), frame.size());
        if (tracked && !roi.empty())
        {
          roi_ = roi;
        }
        else
        {
          tracked = false;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Lost the target");
        }
      }
      else if (!tracker_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "ROI is not defined, please set via GUI or ~set_roi service");
      }

      /// Messages
      opencv_apps::msg::RectArrayStamped output_msg;
      output_msg.header = image_msg->header;
      if (tracked)
      {
        opencv_apps::msg::Rect rect_msg;
        // publish x, y as center of target
        rect_msg.x = roi_.x + roi_.width / 2.0;
        rect_msg.y = roi_.y + roi_.height / 2.0;
        rect_msg.width = roi_.width;
        rect_msg.height = roi_.height;
        output_msg.rects.push_back(rect_msg);
      }

      // copy the input image only when the rect is drawn on it
      cv::Mat result_image = tracker_ ? frame.clone() : frame;
      if (tracker_)
      {
        // blue if tracked, dark blue if lost
        cv::rectangle(result_image, roi_, tracked ? cv::Scalar(255, 0, 0) : cv::Scalar(127, 32, 32), tracked ? 2 : 1, 1);
      }

      /// Show what you got
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        image_size_ = frame.size();
        cv::setMouseCallback(window_name_, onMouse, this);
        window_created_ = true;
        cv::Mat view_image = result_image;
        if (selecting_)
        {
          view_image = result_image.clone();
          cv::rectangle(view_image, selection_, cv::Scalar(0, 0, 255), 2, 1);
        }
        cv::imshow(window_name_, view_image);
        int c = cv::waitKey(1);
        processMouseEvents();
        if (c == 'c' || c == 'C')  // cancel tracking
        {
          tracker_.reset();
          pending_tracker_.reset();
        }
      }

      // Publish the image.
      img_pub_.publish(
          cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::BGR8, result_image).toImageMsg());

      // publish only when the target is set
      if (tracker_)
      {
        msg_pub_->publish(output_msg);
      }
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }
    catch (std::exception& e)  // e.g. cv_bridge::Exception
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s", e.what());
    }
  }

public:
  ~TrackingNodelet()
  {
    // the window may outlive this node, e.g. unloaded from a component container, then its mouse callback must not
    // call this node
    if (window_created_)
    {
      try
      {
        cv::setMouseCallback(window_name_, nullptr, nullptr);
        cv::destroyWindow(window_name_);
      }
      catch (const cv::Exception&)
      {
      }
    }
  }

  explicit TrackingNodelet(const rclcpp::NodeOptions& options) : Nodelet("tracking", options), selecting_(false)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    // with the namespace, not to share the window with another tracking node in the same process
    window_name_ = std::string("Tracking Demo (") + get_fully_qualified_name() + ")";
    window_created_ = false;

    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    tracking_algorithm_ = declare_parameter(
        "tracking_algorithm", 4,
        paramDescriptor("Tracking Algorithm, 0: MIL, 1: BOOSTING, 2: MEDIANFLOW, 3: TLD, 4: KCF, 6: MOSSE, 7: CSRT", 0,
                        7));
    on_set_parameters_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&TrackingNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::RectArrayStamped>("~/output", 1);
    roi_srv_ = create_service<opencv_apps::srv::SetROI>(
        "~/set_roi", std::bind(&TrackingNodelet::setROICb, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Hot keys: ");
    RCLCPP_INFO(get_logger(), "\tc - stop the tracking");
    RCLCPP_INFO(get_logger(), "To start tracking, select the target with mouse, or call ~set_roi service");
    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::TrackingNodelet)
