// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, JSK Lab.
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
*   * Neither the name of the Kei Okada nor the names of its
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

/**
 * @file https://github.com/opencv/opencv/blob/master/samples/cpp/lsd_lines.cpp
 * @brief Sample code showing how to detect line segments using the LineSegmentDetector
 * @author OpenCV team
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/Line.h"
#include "opencv_apps/LineArrayStamped.h"
#include "opencv_apps/LineSegmentDetectorConfig.h"


namespace opencv_apps {
class LineSegmentDetectorNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef line_segment_detector::LineSegmentDetectorConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  cv::Ptr<cv::LineSegmentDetector> lsd_;

  bool debug_view_;

  int lsd_refine_;
  double lsd_scale_;
  double lsd_sigma_scale_;
  double lsd_quant_;
  double lsd_angle_threshold_;
  double lsd_log_eps_;
  double lsd_density_threshold_;
  int lsd_n_bins_;
  double lsd_line_length_threshold_;

  std::string window_name_;
  static bool need_config_update_;

  boost::mutex mutex_;

  void updateLSD() {
    lsd_ = cv::createLineSegmentDetector(lsd_refine_, lsd_scale_,
                                         lsd_sigma_scale_, lsd_quant_,
                                         lsd_angle_threshold_, lsd_log_eps_,
                                         lsd_density_threshold_, lsd_n_bins_);
  }

  void reconfigureCallback(Config &new_config, uint32_t level)
  {
    boost::mutex::scoped_lock lock (mutex_);
    config_ = new_config;

    lsd_refine_ = config_.lsd_refine_type;
    lsd_scale_ = config_.lsd_scale;
    lsd_sigma_scale_ = config_.lsd_sigma_scale;
    lsd_angle_threshold_ = config_.lsd_angle_threshold;
    lsd_log_eps_ = config_.lsd_log_eps;
    lsd_density_threshold_ = config_.lsd_density_threshold;
    lsd_n_bins_ = config_.lsd_n_bins;
    lsd_line_length_threshold_ = config_.lsd_line_length_threshold;

    updateLSD();
  }

  const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    do_work(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    do_work(msg, msg->header.frame_id);
  }

  static void trackbarCallback( int, void* )
  {
    need_config_update_ = true;
  }

  void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
  {
    boost::mutex::scoped_lock lock (mutex_);
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Do the work
      cv::Mat src_gray;
      /// Convert it to gray
      if ( frame.channels() > 1 ) {
        cv::cvtColor( frame, src_gray, cv::COLOR_RGB2GRAY );
      } else {
        src_gray = frame;
      }

      /// Create window
      if( debug_view_) {
        cv::namedWindow( window_name_, cv::WINDOW_AUTOSIZE );
      }

      cv::Mat line_image;

      std::vector<cv::Vec4i> lines;
      std::vector<float> widths;
      std::vector<double> precs;
      std::vector<double> nfas;
      lsd_->detect(src_gray, lines, widths, precs, nfas);
      line_image = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);

      // Messages
      opencv_apps::LineArrayStamped lines_msg;
      lines_msg.header = msg->header;

      // draw lines
      for(size_t i = 0; i < lines.size(); ++i ) {
          int x1 = lines[i][0];
          int y1 = lines[i][1];
          int x2 = lines[i][2];
          int y2 = lines[i][3];
          double line_length = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

          if (line_length >= lsd_line_length_threshold_) {
              cv::line(line_image, cv::Point(x1, y1), cv::Point(x2, y2), 255);
          }

          opencv_apps::Line line_msg;
          line_msg.pt1.x = x1;
          line_msg.pt1.y = y1;
          line_msg.pt2.x = x2;
          line_msg.pt2.y = y2;
          lines_msg.lines.push_back(line_msg);
      }

      if( debug_view_) {
        cv::imshow( window_name_, line_image );
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, line_image).toImageMsg();
      img_pub_.publish(out_img);
      // Publish the detected lines.
      msg_pub_.publish(lines_msg);
    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

  }

  void subscribe()
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", 3, &LineSegmentDetectorNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &LineSegmentDetectorNodelet::imageCallback, this);
  }

  void unsubscribe()
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  virtual void onInit()
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("debug_view", debug_view_, false);

    if (debug_view_) {
      always_subscribe_ = true;
    }

    window_name_ = "Line Segment Detector Demo";

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&LineSegmentDetectorNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    updateLSD();

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::LineArrayStamped>(*pnh_, "lines", 1);

    onInitPostProcess();
  }
};
bool LineSegmentDetectorNodelet::need_config_update_ = false;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::LineSegmentDetectorNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(line_segment_detector::LineSegmentDetectorNodelet, nodelet::Nodelet);
