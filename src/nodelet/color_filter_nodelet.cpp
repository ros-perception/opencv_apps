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
#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/RGBColorFilterConfig.h"
#include "opencv_apps/HLSColorFilterConfig.h"
#include "opencv_apps/HSVColorFilterConfig.h"

namespace opencv_apps {
class RGBColorFilter;
class HLSColorFilter;
class HSVColorFilter;

template <typename Config>
class ColorFilterNodelet : public opencv_apps::Nodelet
{
  friend class RGBColorFilter;
  friend class HLSColorFilter;

protected:
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  bool debug_view_;

  std::string window_name_;

  cv::Scalar lower_color_range_;
  cv::Scalar upper_color_range_;

  boost::mutex mutex_;

  virtual void reconfigureCallback(Config &new_config, uint32_t level) = 0;

  virtual void filter(const cv::Mat& input_image, cv::Mat& output_image) = 0;

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

  void do_work(const sensor_msgs::ImageConstPtr& image_msg, const std::string input_frame_from_msg)
  {
    // Work on the image.
    try
      {
        // Convert the image into something opencv can handle.
        cv::Mat frame = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image;

        // Do the work
        cv::Mat out_frame;
        filter(frame, out_frame);

        /// Create window
        if( debug_view_) {
          cv::namedWindow( window_name_, cv::WINDOW_AUTOSIZE );
        }

        std::string new_window_name;

        if( debug_view_) {
          if (window_name_ != new_window_name) {
            cv::destroyWindow(window_name_);
            window_name_ = new_window_name;
          }
          cv::imshow( window_name_, out_frame );
          int c = cv::waitKey(1);
        }

        // Publish the image.
        sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::MONO8, out_frame).toImageMsg();
        img_pub_.publish(out_img);
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
      cam_sub_ = it_->subscribeCamera("image", 3, &ColorFilterNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &ColorFilterNodelet::imageCallback, this);
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

    window_name_ = "ColorFilter Demo";

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ColorFilterNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);

    onInitPostProcess();
  }
};

class RGBColorFilterNodelet
  : public ColorFilterNodelet<opencv_apps::RGBColorFilterConfig> {
protected:
  int r_min_;
  int r_max_;
  int b_min_;
  int b_max_;
  int g_min_;
  int g_max_;

  virtual void reconfigureCallback(opencv_apps::RGBColorFilterConfig& config,
                                   uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    r_max_ = config.r_limit_max;
    r_min_ = config.r_limit_min;
    g_max_ = config.g_limit_max;
    g_min_ = config.g_limit_min;
    b_max_ = config.b_limit_max;
    b_min_ = config.b_limit_min;
    updateCondition();
  }

  virtual void updateCondition() {
    if (r_max_ < r_min_) std::swap(r_max_, r_min_);
    if (g_max_ < g_min_) std::swap(g_max_, g_min_);
    if (b_max_ < b_min_) std::swap(b_max_, b_min_);
    lower_color_range_ = cv::Scalar(b_min_, g_min_, r_min_);
    upper_color_range_ = cv::Scalar(b_max_, g_max_, r_max_);
  }

  virtual void filter(const cv::Mat& input_image, cv::Mat& output_image) {
    cv::inRange(input_image, lower_color_range_, upper_color_range_,
                output_image);
  }

private:
  virtual void onInit() {
    r_max_ = 255;
    r_min_ = 0;
    g_max_ = 255;
    g_min_ = 0;
    b_max_ = 255;
    b_min_ = 0;

    ColorFilterNodelet::onInit();
  }
};

class HLSColorFilterNodelet
  : public ColorFilterNodelet<opencv_apps::HLSColorFilterConfig> {
protected:
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int l_min_;
  int l_max_;

  virtual void reconfigureCallback(opencv_apps::HLSColorFilterConfig& config,
                                   uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    h_max_ = config.h_limit_max;
    h_min_ = config.h_limit_min;
    s_max_ = config.s_limit_max;
    s_min_ = config.s_limit_min;
    l_max_ = config.l_limit_max;
    l_min_ = config.l_limit_min;
    updateCondition();
  }

  virtual void updateCondition() {
    if (s_max_ < s_min_) std::swap(s_max_, s_min_);
    if (l_max_ < l_min_) std::swap(l_max_, l_min_);
    lower_color_range_ = cv::Scalar(h_min_/2, l_min_, s_min_, 0);
    upper_color_range_ = cv::Scalar(h_max_/2, l_max_, s_max_, 0);
  }

  virtual void filter(const cv::Mat& input_image, cv::Mat& output_image) {
    cv::Mat hls_image;
    cv::cvtColor(input_image, hls_image, cv::COLOR_BGR2HLS);
    if ( lower_color_range_[0] < upper_color_range_[0] ) {
      cv::inRange(hls_image, lower_color_range_, upper_color_range_,
                  output_image);
    }else {
      cv::Scalar lower_color_range_0 = cv::Scalar(       0, l_min_, s_min_, 0);
      cv::Scalar upper_color_range_0 = cv::Scalar(h_max_/2, l_max_, s_max_, 0);
      cv::Scalar lower_color_range_360 = cv::Scalar(h_min_/2, l_min_, s_min_, 0);
      cv::Scalar upper_color_range_360 = cv::Scalar(   360/2, l_max_, s_max_, 0);
      cv::Mat output_image_0, output_image_360;
      cv::inRange(hls_image, lower_color_range_0, upper_color_range_0, output_image_0);
      cv::inRange(hls_image, lower_color_range_360, upper_color_range_360, output_image_360);
      output_image = output_image_0 | output_image_360;
    }
  }

public:
  virtual void onInit() {
    h_max_ = 360;
    h_min_ = 0;
    s_max_ = 256;
    s_min_ = 0;
    l_max_ = 256;
    l_min_ = 0;

    ColorFilterNodelet::onInit();
  }
};

class HSVColorFilterNodelet
  : public ColorFilterNodelet<opencv_apps::HSVColorFilterConfig> {
protected:
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int v_min_;
  int v_max_;

  virtual void reconfigureCallback(opencv_apps::HSVColorFilterConfig& config,
                                   uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    h_max_ = config.h_limit_max;
    h_min_ = config.h_limit_min;
    s_max_ = config.s_limit_max;
    s_min_ = config.s_limit_min;
    v_max_ = config.v_limit_max;
    v_min_ = config.v_limit_min;
    updateCondition();
  }

  virtual void updateCondition() {
    if (s_max_ < s_min_) std::swap(s_max_, s_min_);
    if (v_max_ < v_min_) std::swap(v_max_, v_min_);
    lower_color_range_ = cv::Scalar(h_min_/2, s_min_, v_min_, 0);
    upper_color_range_ = cv::Scalar(h_max_/2, s_max_, v_max_, 0);
  }

  virtual void filter(const cv::Mat& input_image, cv::Mat& output_image) {
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
    if ( lower_color_range_[0] < upper_color_range_[0] ) {
      cv::inRange(hsv_image, lower_color_range_, upper_color_range_,
                  output_image);
    }else {
      cv::Scalar lower_color_range_0 = cv::Scalar(       0, s_min_, v_min_, 0);
      cv::Scalar upper_color_range_0 = cv::Scalar(h_max_/2, s_max_, v_max_, 0);
      cv::Scalar lower_color_range_360 = cv::Scalar(h_min_/2, s_min_, v_min_, 0);
      cv::Scalar upper_color_range_360 = cv::Scalar(   360/2, s_max_, v_max_, 0);
      cv::Mat output_image_0, output_image_360;
      cv::inRange(hsv_image, lower_color_range_0, upper_color_range_0, output_image_0);
      cv::inRange(hsv_image, lower_color_range_360, upper_color_range_360, output_image_360);
      output_image = output_image_0 | output_image_360;
    }
  }

public:
  virtual void onInit() {
    h_max_ = 360;
    h_min_ = 0;
    s_max_ = 256;
    s_min_ = 0;
    v_max_ = 256;
    v_min_ = 0;

    ColorFilterNodelet::onInit();
  }
};

}

#include <pluginlib/class_list_macros.h>
typedef opencv_apps::RGBColorFilterNodelet RGBColorFilterNodelet;
typedef opencv_apps::HLSColorFilterNodelet HLSColorFilterNodelet;
typedef opencv_apps::HSVColorFilterNodelet HSVColorFilterNodelet;
PLUGINLIB_EXPORT_CLASS(opencv_apps::RGBColorFilterNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(opencv_apps::HLSColorFilterNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(opencv_apps::HSVColorFilterNodelet, nodelet::Nodelet);
