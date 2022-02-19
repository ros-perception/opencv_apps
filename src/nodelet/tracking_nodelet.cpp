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
 * This is a demo of tracking image processing,
 */

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_apps/nodelet.h"
#include "opencv_apps/RectArrayStamped.h"
#include "opencv_apps/SetImages.h"
#include "opencv_apps/TrackingConfig.h"

#include <boost/filesystem.hpp>

#include <dynamic_reconfigure/server.h>

namespace opencv_apps
{
class TrackingNodelet : public opencv_apps::Nodelet
{
  ////////////////////////////////////////////////////////
  // Dynamic Reconfigure
  ////////////////////////////////////////////////////////
  typedef opencv_apps::TrackingConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_;

  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;
  ros::ServiceServer roi_srv_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  boost::mutex mutex_;
  int tracking_algorithm_;

  cv::Ptr<cv::Tracker> tracker_;
  cv::TrackerKCF::Params params_;
  cv::Rect2d roi_;

public:
  static cv::Mat frame_;

  // copied from opencv/modules/highgui/src/roiSelector.cpp
  struct HandlerT
  {
    // basic parameters
    bool isDrawing;
    cv::Rect2d box;
    cv::Mat image;
    cv::Point2f startPos;

    // initializer list
    HandlerT() : isDrawing(false){};
  } selectorParams;

  static void onMouse(int event, int x, int y, int flags, void* param)
  {
    TrackingNodelet* self = static_cast<TrackingNodelet*>(param);
    self->opencvMouseCallback(event, x, y, flags);
  }

  void opencvMouseCallback(int event, int x, int y, int /*unused*/)
  {
    // copied from opencv/modules/highgui/src/roiSelector.cpp
    switch (event)
    {
      // update the selected bounding box
      case cv::EVENT_MOUSEMOVE:
        if (selectorParams.isDrawing)
        {
          {
            // limit x and y to imageSize
            int lx = std::min(std::max(x, 0), imageSize.width);
            int by = std::min(std::max(y, 0), imageSize.height);
            selectorParams.box.width = std::abs(lx - selectorParams.startPos.x);
            selectorParams.box.height = std::abs(by - selectorParams.startPos.y);
            selectorParams.box.x = std::min((float)lx, selectorParams.startPos.x);
            selectorParams.box.y = std::min((float)by, selectorParams.startPos.y);
          }
        }
        break;

        // start to select the bounding box
      case cv::EVENT_LBUTTONDOWN:
        selectorParams.isDrawing = true;
        selectorParams.box = cv::Rect2d(x, y, 0, 0);
        selectorParams.startPos = cv::Point2f((float)x, (float)y);
        break;

        // cleaning up the selected bounding box
      case cv::EVENT_LBUTTONUP:
        selectorParams.isDrawing = false;
        if (selectorParams.box.width < 0)
        {
          selectorParams.box.x += selectorParams.box.width;
          selectorParams.box.width *= -1;
        }
        if (selectorParams.box.height < 0)
        {
          selectorParams.box.y += selectorParams.box.height;
          selectorParams.box.height *= -1;
        }
        initializeTracker(tracking_algorithm_, selectorParams.box);
        break;
    }
  }
  // save the keypressed character
  cv::Size imageSize;

  bool setROICb(opencv_apps::SetImages::Request& request, opencv_apps::SetImages::Response& response)
  {
    if (frame_.empty() && request.images.empty())
    {
      response.ok = false;
      response.error =
          "Can not set ROI because of missing template image, we need to set request.images or subscribe input image";
      ROS_ERROR_STREAM(response.error);
      return true;
    }
    if (request.rects.size() != 1)
    {
      response.ok = false;
      response.error = "tracking_nodelet does not supoprt multiple ROI's";
      ROS_ERROR_STREAM(response.error);
    }
    else
    {
      cv::Rect2d roi(request.rects[0].x - request.rects[0].width / 2, request.rects[0].y - request.rects[0].height / 2,
                     request.rects[0].width, request.rects[0].height);
      if (!request.images.empty())
      {
        sensor_msgs::Image img_msg = request.images[0];
        cv::Mat frame = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
        initializeTracker(tracking_algorithm_, roi, frame);
      }
      else
      {
        initializeTracker(tracking_algorithm_, roi);
      }
      response.ok = true;
    }
    return true;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &TrackingNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &TrackingNodelet::imageCallback, this);
  }

  void unsubscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  void reconfigureCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    tracking_algorithm_ = config.tracking_algorithm;
    initializeTracker(tracking_algorithm_, roi_);
  }

  void initializeTracker(int tracking_algorithm, const cv::Rect& roi, const cv::Mat& frame = frame_)
  {
    if (roi.empty())
    {
      return;
    }
    switch (tracking_algorithm)
    {
      case opencv_apps::Tracking_MIL:
        tracker_ = cv::TrackerMIL::create();
        ROS_INFO("Create MIL (Multiple Instance Learning) tracker");
        break;
      case opencv_apps::Tracking_BOOSTING:
        tracker_ = cv::TrackerBoosting::create();
        ROS_INFO("Create On-line version of the AdaBoost tracker");
        break;
      case opencv_apps::Tracking_MEDIANFLOW:
        tracker_ = cv::TrackerMedianFlow::create();
        ROS_INFO("Create Median Flow tracker");
        break;
      case opencv_apps::Tracking_TLD:
        tracker_ = cv::TrackerTLD::create();
        ROS_INFO("Create TLD (Tracking, learning and detection) tracker");
        break;
      case opencv_apps::Tracking_KCF:
        tracker_ = cv::TrackerKCF::create(params_);
        ROS_INFO("Create KCF (Kernelized Correlation Filter) tracker");
        break;
      case opencv_apps::Tracking_GOTURN:
        if (boost::filesystem::exists("goturn.caffemodel") && boost::filesystem::exists("goturn.prototxt"))
        {
          boost::filesystem::exists("goturn.caffemodel");
          tracker_ = cv::TrackerGOTURN::create();
          ROS_INFO("Create GOTURN (Generic Object Tracking Using Regression Networks) tracker");
        }
        //catch (boost::system::error_code& e) {
        //          ROS_ERROR_STREAM("Faield to Create GOTURN tracker : " << e);
        //        }
        //        catch (std::exception& e) {
        else
        {
          //ROS_ERROR_STREAM("Faield to Create GOTURN tracker : " << e.what());
          ROS_ERROR_STREAM("GOTURN tracker need Coffe model(goturn.caffemodel, goturn.prototxt) under "
                           << boost::filesystem::current_path());
          ROS_ERROR_STREAM("Download caffe model from https://github.com/Mogball/goturn-files");
          ROS_ERROR_STREAM(" and \"sed -i '/^input_dim:/ s/^/# /' goturn.prototxt\" as described in "
                           "https://github.com/opencv/opencv_contrib/issues/2426");
          ROS_ERROR_STREAM(" until https://github.com/opencv/opencv/pull/16617 released");
        }
        break;
      case opencv_apps::Tracking_MOSSE:
        tracker_ = cv::TrackerMOSSE::create();
        ROS_INFO("Create MOSSE (Minimum Output Sum of Squared Error) tracker");
        break;
    }
    ROS_INFO_STREAM("       on ROI (center-x: " << roi.x + roi.width / 2 << ", center-y: " << roi.y + roi.height / 2
                                                << ", width: " << roi.width << ", height: " << roi.height << ")");
    tracker_->init(frame, roi);
    roi_ = roi;
  }

  void doWork(const sensor_msgs::Image::ConstPtr& image_msg, const std::string& input_frame_from_msg)
  {
    try
    {
      frame_ = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;

      // do not track if ROI was not selected
      bool retval = false;
      if (roi_.empty())
      {
        ROS_WARN_THROTTLE(3, "ROI is not defined, please set via GUI or /set_roi message interface");
      }
      else
      {
        /// Tracking
        boost::mutex::scoped_lock lock(mutex_);
        retval = tracker_->update(frame_, roi_);

        // keep roi within image
        roi_ = roi_ & cv::Rect2d(cv::Point(0, 0), frame_.size());
      }

      /// Messages
      opencv_apps::RectArrayStamped output_msg;
      output_msg.header = image_msg->header;

      cv::Mat result_image;
      result_image = frame_.clone();
      /// Show what you got
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        imageSize = frame_.size();
        cv::setMouseCallback(window_name_, onMouse, (void*)this);

        if (selectorParams.isDrawing)
        {
          cv::rectangle(result_image, selectorParams.box, cv::Scalar(0, 0, 255), 2, 1);
        }
        if (!roi_.empty())
        {
          cv::rectangle(result_image, roi_, retval ? cv::Scalar(255, 0, 0) : cv::Scalar(127, 32, 32), retval ? 2 : 1, 1);
          output_msg.rects.resize(1);
          // publish x, y as center of target
          output_msg.rects[0].x = roi_.x + roi_.width / 2;
          output_msg.rects[0].y = roi_.y + roi_.height / 2;
          output_msg.rects[0].width = roi_.width;
          output_msg.rects[0].height = roi_.height;
        }
        cv::imshow(window_name_, result_image);
        int c = cv::waitKey(1);
        if (c == 'c' || c == 'C')  // cancel selection
        {
          roi_ = cv::Rect();
        }
      }

      // Publish the image.
      img_pub_.publish(
          cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::BGR8, result_image).toImageMsg());

      // publish only when ROI is defiend
      if (!roi_.empty())
      {
        msg_pub_.publish(output_msg);
      }
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  }

public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("debug_view", debug_view_, false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }

    window_name_ = "Tracking Demo (" + ros::this_node::getName() + ")";
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&TrackingNodelet::reconfigureCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::RectArrayStamped>(*pnh_, "output", 1);
    roi_srv_ = pnh_->advertiseService("set_roi", &TrackingNodelet::setROICb, this);
    onInitPostProcess();
  }
};
cv::Mat TrackingNodelet::frame_;
}  // namespace opencv_apps

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::TrackingNodelet, nodelet::Nodelet);
