// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Kei Okada
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

// https://github.com/opencv/opencv/blob/4.x/samples/dnn/dasiamrpn_tracker.cpp
// https://github.com/opencv/opencv/blob/4.x/samples/dnn/nanotrack_tracker.cpp
// https://github.com/opencv/opencv_zoo/tree/main/models/object_tracking_vittrack
/**
 * This is a demo of the DNN based trackers of OpenCV: DaSiamRPN, NanoTrack and VitTrack. Select the target with
 * mouse, or call ~set_roi service, as the tracking node.
 */

#include "opencv_apps/tracker_nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/core/version.hpp>
// cv::TrackerDaSiamRPN, cv::TrackerNano (OpenCV 4.7 or later) and cv::TrackerVit (OpenCV 4.9 or later) are in the
// video module, with the dnn module
#include <opencv2/video/tracking.hpp>

#include <string>
#include <tuple>
#include <vector>

#define OPENCV_APPS_CV_VERSION_GE(major, minor)                                                                        \
  (CV_VERSION_MAJOR > (major) || (CV_VERSION_MAJOR == (major) && CV_VERSION_MINOR >= (minor)))

namespace opencv_apps
{
enum
{
  DNNTracking_AUTO = -1,
  DNNTracking_DaSiamRPN = 0,
  DNNTracking_NanoTrack = 1,
  DNNTracking_VitTrack = 2
};

// the default models (onnx), downloaded to ~/.ros/opencv_apps/models/ at the first time
// https://github.com/opencv/opencv/blob/4.x/samples/dnn/dasiamrpn_tracker.cpp
const char* const DASIAMRPN_MODEL_URL = "https://www.dropbox.com/s/rr1lk9355vzolqv/dasiamrpn_model.onnx?dl=1";
const char* const DASIAMRPN_KERNEL_CLS1_URL =
    "https://www.dropbox.com/s/qvmtszx5h339a0w/dasiamrpn_kernel_cls1.onnx?dl=1";
const char* const DASIAMRPN_KERNEL_R1_URL = "https://www.dropbox.com/s/999cqx5zrfi7w4p/dasiamrpn_kernel_r1.onnx?dl=1";
// NanoTrack v2 of https://github.com/HonglinChu/SiamTrackers (Apache 2.0), as samples/dnn/nanotrack_tracker.cpp, the
// commit is fixed not to change them
const char* const NANOTRACK_BACKBONE_URL = "https://raw.githubusercontent.com/HonglinChu/SiamTrackers/"
                                           "8fe6791fecf13b95e6f19770af8963a9fc98759b/NanoTrack/models/nanotrackv2/"
                                           "nanotrack_backbone_sim.onnx";
const char* const NANOTRACK_NECKHEAD_URL = "https://raw.githubusercontent.com/HonglinChu/SiamTrackers/"
                                           "8fe6791fecf13b95e6f19770af8963a9fc98759b/NanoTrack/models/nanotrackv2/"
                                           "nanotrack_head_sim.onnx";
// https://github.com/opencv/opencv_zoo/tree/main/models/object_tracking_vittrack (Apache 2.0), the commit is fixed
const char* const VITTRACK_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/"
                                       "48b10fa505aeb51fbd127fc60a5c660040e823b7/models/object_tracking_vittrack/"
                                       "object_tracking_vittrack_2023sep.onnx";

class DNNTrackingNodelet : public opencv_apps::TrackerNodelet
{
  // the parameters of the tracker, the models are paths or URLs, empty for the default models
  struct Config
  {
    int tracking_algorithm;
    std::string backend;
    std::string dasiamrpn_model, dasiamrpn_kernel_cls1, dasiamrpn_kernel_r1;
    std::string nanotrack_backbone, nanotrack_neckhead;
    std::string vittrack_model;

    bool operator==(const Config& c) const
    {
      return std::tie(tracking_algorithm, backend, dasiamrpn_model, dasiamrpn_kernel_cls1, dasiamrpn_kernel_r1,
                      nanotrack_backbone, nanotrack_neckhead, vittrack_model) ==
             std::tie(c.tracking_algorithm, c.backend, c.dasiamrpn_model, c.dasiamrpn_kernel_cls1,
                      c.dasiamrpn_kernel_r1, c.nanotrack_backbone, c.nanotrack_neckhead, c.vittrack_model);
    }
  };
  Config config_;
  // the error of prepareTracker(), not to download the models again with mutex_ locked
  std::string prepare_error_;

  // the newest tracker of this OpenCV for tracking_algorithm -1
  static int autoAlgorithm()
  {
#if OPENCV_APPS_CV_VERSION_GE(4, 9)
    return DNNTracking_VitTrack;
#elif OPENCV_APPS_CV_VERSION_GE(4, 7)
    return DNNTracking_NanoTrack;
#else
    return DNNTracking_DaSiamRPN;
#endif
  }

  // path of the model file of the parameter (or the default URL if it is empty), downloaded if it is a URL
  std::string model(const std::string& parameter, const char* default_url, std::string& error)
  {
    std::string file = modelFile(parameter.empty() ? std::string(default_url) : parameter, get_logger());
    if (file.empty())
      error = "Failed to get the model " + (parameter.empty() ? std::string(default_url) : parameter);
    return file;
  }

  cv::Ptr<cv::Tracker> createTracker(const Config& config, std::string& error)
  {
    int backend, target;
    if (!dnnBackend(config.backend, backend, target, get_logger()))
    {
      error = "backend " + config.backend + " is not available";
      return cv::Ptr<cv::Tracker>();
    }
    int tracking_algorithm =
        config.tracking_algorithm == DNNTracking_AUTO ? autoAlgorithm() : config.tracking_algorithm;
    try
    {
      switch (tracking_algorithm)
      {
        case DNNTracking_DaSiamRPN:
        {
          cv::TrackerDaSiamRPN::Params params;
          if ((params.model = model(config.dasiamrpn_model, DASIAMRPN_MODEL_URL, error)).empty() ||
              (params.kernel_cls1 = model(config.dasiamrpn_kernel_cls1, DASIAMRPN_KERNEL_CLS1_URL, error)).empty() ||
              (params.kernel_r1 = model(config.dasiamrpn_kernel_r1, DASIAMRPN_KERNEL_R1_URL, error)).empty())
            return cv::Ptr<cv::Tracker>();
          params.backend = backend;
          params.target = target;
          RCLCPP_INFO(get_logger(), "Create DaSiamRPN (Distractor-aware Siamese Region Proposal Network) tracker");
          return cv::TrackerDaSiamRPN::create(params);
        }
#if OPENCV_APPS_CV_VERSION_GE(4, 7)
        case DNNTracking_NanoTrack:
        {
          cv::TrackerNano::Params params;
          if ((params.backbone = model(config.nanotrack_backbone, NANOTRACK_BACKBONE_URL, error)).empty() ||
              (params.neckhead = model(config.nanotrack_neckhead, NANOTRACK_NECKHEAD_URL, error)).empty())
            return cv::Ptr<cv::Tracker>();
          params.backend = backend;
          params.target = target;
          RCLCPP_INFO(get_logger(), "Create NanoTrack tracker");
          return cv::TrackerNano::create(params);
        }
#endif
#if OPENCV_APPS_CV_VERSION_GE(4, 9)
        case DNNTracking_VitTrack:
        {
          cv::TrackerVit::Params params;
          if ((params.net = model(config.vittrack_model, VITTRACK_MODEL_URL, error)).empty())
            return cv::Ptr<cv::Tracker>();
          params.backend = backend;
          params.target = target;
          RCLCPP_INFO(get_logger(), "Create VitTrack (Vision Transformer) tracker");
          return cv::TrackerVit::create(params);
        }
#endif
        default:
          error = "tracking_algorithm " + std::to_string(tracking_algorithm) +
                  " is not available in this build (OpenCV " + CV_VERSION +
                  "): NanoTrack needs OpenCV 4.7 or later, and VitTrack needs OpenCV 4.9 or later";
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
    return createTracker(config_, error);
  }

  // the parameters of the tracker with parameters, which are changed from config_
  Config configWith(const std::vector<rclcpp::Parameter>& parameters) const
  {
    Config config = config_;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      const std::string& name = parameter.get_name();
      if (name == "tracking_algorithm")
        config.tracking_algorithm = parameter.as_int();
      else if (name == "backend")
        config.backend = parameter.as_string();
      else if (name == "dasiamrpn_model")
        config.dasiamrpn_model = parameter.as_string();
      else if (name == "dasiamrpn_kernel_cls1")
        config.dasiamrpn_kernel_cls1 = parameter.as_string();
      else if (name == "dasiamrpn_kernel_r1")
        config.dasiamrpn_kernel_r1 = parameter.as_string();
      else if (name == "nanotrack_backbone")
        config.nanotrack_backbone = parameter.as_string();
      else if (name == "nanotrack_neckhead")
        config.nanotrack_neckhead = parameter.as_string();
      else if (name == "vittrack_model")
        config.vittrack_model = parameter.as_string();
    }
    return config;
  }

  // download the models of the new parameters, which are used by setTrackerParameters() with mutex_ locked
  void prepareTracker(const std::vector<rclcpp::Parameter>& parameters) override
  {
    prepare_error_.clear();
    Config config = configWith(parameters);
    if (config == config_)
      return;
    std::string& error = prepare_error_;
    switch (config.tracking_algorithm == DNNTracking_AUTO ? autoAlgorithm() : config.tracking_algorithm)
    {
      case DNNTracking_DaSiamRPN:
        model(config.dasiamrpn_model, DASIAMRPN_MODEL_URL, error);
        model(config.dasiamrpn_kernel_cls1, DASIAMRPN_KERNEL_CLS1_URL, error);
        model(config.dasiamrpn_kernel_r1, DASIAMRPN_KERNEL_R1_URL, error);
        break;
      case DNNTracking_NanoTrack:
        model(config.nanotrack_backbone, NANOTRACK_BACKBONE_URL, error);
        model(config.nanotrack_neckhead, NANOTRACK_NECKHEAD_URL, error);
        break;
      case DNNTracking_VitTrack:
        model(config.vittrack_model, VITTRACK_MODEL_URL, error);
        break;
    }
  }

  bool setTrackerParameters(const std::vector<rclcpp::Parameter>& parameters, cv::Ptr<cv::Tracker>& tracker,
                            std::string& error) override
  {
    Config config = configWith(parameters);
    if (config == config_)
      return true;
    // reject the parameters with which the tracker cannot be created, e.g. a model which cannot be downloaded or
    // read
    if (!prepare_error_.empty())
    {
      error = prepare_error_;
      return false;
    }
    tracker = createTracker(config, error);
    if (!tracker)
      return false;
    config_ = config;
    return true;
  }

public:
  explicit DNNTrackingNodelet(const rclcpp::NodeOptions& options)
    : TrackerNodelet("dnn_tracking", "DNN Tracking Demo", options)
  {
    config_.tracking_algorithm = declare_parameter(
        "tracking_algorithm", static_cast<int>(DNNTracking_AUTO),
        paramDescriptor("Tracking Algorithm, -1: the newest one of this OpenCV (VitTrack, NanoTrack or DaSiamRPN), 0: "
                        "DaSiamRPN, 1: NanoTrack (OpenCV 4.7 or later), 2: VitTrack (OpenCV 4.9 or later)",
                        -1, 2));
    config_.backend = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    const std::string model_description =
        ", a path or URL (downloaded to ~/.ros/opencv_apps/models/ at the first time), empty for the default ";
    config_.dasiamrpn_model = declare_parameter(
        "dasiamrpn_model", std::string(""),
        paramDescriptor("dasiamrpn_model.onnx of DaSiamRPN" + model_description + DASIAMRPN_MODEL_URL));
    config_.dasiamrpn_kernel_cls1 = declare_parameter(
        "dasiamrpn_kernel_cls1", std::string(""),
        paramDescriptor("dasiamrpn_kernel_cls1.onnx of DaSiamRPN" + model_description + DASIAMRPN_KERNEL_CLS1_URL));
    config_.dasiamrpn_kernel_r1 = declare_parameter(
        "dasiamrpn_kernel_r1", std::string(""),
        paramDescriptor("dasiamrpn_kernel_r1.onnx of DaSiamRPN" + model_description + DASIAMRPN_KERNEL_R1_URL));
    config_.nanotrack_backbone =
        declare_parameter("nanotrack_backbone", std::string(""),
                          paramDescriptor("Backbone model of NanoTrack" + model_description + NANOTRACK_BACKBONE_URL));
    config_.nanotrack_neckhead = declare_parameter(
        "nanotrack_neckhead", std::string(""),
        paramDescriptor("Neck and head model of NanoTrack" + model_description + NANOTRACK_NECKHEAD_URL));
    config_.vittrack_model =
        declare_parameter("vittrack_model", std::string(""),
                          paramDescriptor("Model of VitTrack" + model_description + VITTRACK_MODEL_URL));
    startTracking();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNTrackingNodelet)
