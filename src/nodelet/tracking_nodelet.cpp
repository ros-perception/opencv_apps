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

#include "opencv_apps/tracker_nodelet.h"

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

class TrackingNodelet : public opencv_apps::TrackerNodelet
{
  int tracking_algorithm_;

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

  cv::Ptr<cv::Tracker> createTracker(std::string& error) override
  {
    return createTracker(tracking_algorithm_, error);
  }

  bool setTrackerParameters(const std::vector<rclcpp::Parameter>& parameters, cv::Ptr<cv::Tracker>& tracker,
                            std::string& error) override
  {
    int tracking_algorithm = tracking_algorithm_;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "tracking_algorithm")
        tracking_algorithm = parameter.as_int();
    }
    if (tracking_algorithm == tracking_algorithm_)
      return true;
    // reject the tracking algorithm which is not available
    tracker = createTracker(tracking_algorithm, error);
    if (!tracker)
      return false;
    tracking_algorithm_ = tracking_algorithm;
    return true;
  }

public:
  explicit TrackingNodelet(const rclcpp::NodeOptions& options) : TrackerNodelet("tracking", "Tracking Demo", options)
  {
    tracking_algorithm_ = declare_parameter(
        "tracking_algorithm", 4,
        paramDescriptor("Tracking Algorithm, 0: MIL, 1: BOOSTING, 2: MEDIANFLOW, 3: TLD, 4: KCF, 6: MOSSE, 7: CSRT", 0,
                        7));
    startTracking();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::TrackingNodelet)
