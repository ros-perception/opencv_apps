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
#include "opencv_apps/HSIColorFilterConfig.h"

namespace color_filter {
class RGBColorFilter;
class HSIColorFilter;

template <typename Config>
class ColorFilterNodelet : public opencv_apps::Nodelet
{
  friend class RGBColorFilter;
  friend class HSIColorFilter;

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
  : public ColorFilterNodelet<color_filter::RGBColorFilterConfig> {
protected:
  int r_min_;
  int r_max_;
  int b_min_;
  int b_max_;
  int g_min_;
  int g_max_;

  virtual void reconfigureCallback(color_filter::RGBColorFilterConfig& config,
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

class HSIColorFilterNodelet
  : public ColorFilterNodelet<color_filter::HSIColorFilterConfig> {
protected:
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int i_min_;
  int i_max_;

  virtual void reconfigureCallback(color_filter::HSIColorFilterConfig& config,
                                   uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    h_max_ = config.h_limit_max;
    h_min_ = config.h_limit_min;
    s_max_ = config.s_limit_max;
    s_min_ = config.s_limit_min;
    i_max_ = config.i_limit_max;
    i_min_ = config.i_limit_min;
    updateCondition();
  }

  virtual void updateCondition() {
    if (h_max_ < h_min_) std::swap(h_max_, h_min_);
    if (s_max_ < s_min_) std::swap(s_max_, s_min_);
    if (i_max_ < i_min_) std::swap(i_max_, i_min_);
    lower_color_range_ = cv::Scalar(h_min_, s_min_, i_min_, 0);
    upper_color_range_ = cv::Scalar(h_max_, s_max_, i_max_, 0);
  }

  virtual void filter(const cv::Mat& input_image, cv::Mat& output_image) {
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_image, lower_color_range_, upper_color_range_,
                output_image);
  }

public:
  virtual void onInit() {
    h_max_ = 255;
    h_min_ = 0;
    s_max_ = 255;
    s_min_ = 0;
    i_max_ = 255;
    i_min_ = 0;

    ColorFilterNodelet::onInit();
  }
};

}

#include <pluginlib/class_list_macros.h>
typedef color_filter::RGBColorFilterNodelet RGBColorFilterNodelet;
typedef color_filter::HSIColorFilterNodelet HSIColorFilterNodelet;
PLUGINLIB_EXPORT_CLASS(color_filter::RGBColorFilterNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(color_filter::HSIColorFilterNodelet, nodelet::Nodelet);
