// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/ImgProc/Smoothing.cpp
/**
 * This is a demo of smoothing filters,
 */

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv_apps/nodelet.h"
#include "opencv_apps/NormalizedBlockFilterConfig.h"
#include "opencv_apps/BilateralFilterConfig.h"
#include "opencv_apps/GaussianFilterConfig.h"
#include "opencv_apps/MedianFilterConfig.h"

#include <dynamic_reconfigure/server.h>

namespace smoothing_filter {
  class NormalizedBlockFilterNodelet;
  class BilateralFilterNodelet;
  class GaussianFilterNodelet;
  class MedianFilterNodelet;

  template <typename Config>
  class SmoothingFilterNodelet : public opencv_apps::Nodelet {
    friend class NormalizedBlockFilterNodelet;
    friend class BilateralFilterNodelet;
    friend class GaussianFilterNodelet;
    friend class MedianFilterNodelet;

  protected:
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void reconfigureCallback(Config& config, uint32_t level) = 0;
    virtual void filter(const sensor_msgs::Image::ConstPtr& input_image_msg);
    virtual void process(const cv::Mat& input_image,
                         const std_msgs::Header& header) = 0;

    image_transport::Publisher pub_;
    image_transport::Subscriber sub_;

    boost::shared_ptr<dynamic_reconfigure::Server<Config> > reconfigure_server_;
    boost::shared_ptr<image_transport::ImageTransport> it_;

    boost::mutex mutex_;

  private:
    virtual void onInit();
  };

  class NormalizedBlockFilterNodelet
    : public SmoothingFilterNodelet<
        smoothing_filter::NormalizedBlockFilterConfig> {
  protected:
    int kernel_size_;
    virtual void reconfigureCallback(
      smoothing_filter::NormalizedBlockFilterConfig& config, uint32_t level);
    virtual void process(const cv::Mat& input_image,
                         const std_msgs::Header& header);

  private:
    virtual void onInit();
  };

  class BilateralFilterNodelet
    : public SmoothingFilterNodelet<smoothing_filter::BilateralFilterConfig> {
  protected:
    int filter_size_;
    double sigma_color_;
    double sigma_space_;
    virtual void reconfigureCallback(
      smoothing_filter::BilateralFilterConfig& config, uint32_t level);
    virtual void process(const cv::Mat& input_image,
                         const std_msgs::Header& header);

  private:
    virtual void onInit();
  };

  class GaussianFilterNodelet
    : public SmoothingFilterNodelet<smoothing_filter::GaussianFilterConfig> {
  protected:
    int kernel_size_;
    double sigma_x_;
    double sigma_y_;
    virtual void reconfigureCallback(
      smoothing_filter::GaussianFilterConfig& config, uint32_t level);
    virtual void process(const cv::Mat& input_image,
                         const std_msgs::Header& header);

  private:
    virtual void onInit();
  };

  class MedianFilterNodelet
    : public SmoothingFilterNodelet<smoothing_filter::MedianFilterConfig> {
  protected:
    int kernel_size_;
    virtual void reconfigureCallback(
      smoothing_filter::MedianFilterConfig& config, uint32_t level);
    virtual void process(const cv::Mat& input_image,
                         const std_msgs::Header& header);

  private:
    virtual void onInit();
  };

  /*** NormalizedBlock Filter ***/
  void NormalizedBlockFilterNodelet::onInit() {
    kernel_size_ = 0;
    SmoothingFilterNodelet::onInit();
  }

  void NormalizedBlockFilterNodelet::reconfigureCallback(
    smoothing_filter::NormalizedBlockFilterConfig& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    kernel_size_ = config.kernel_size * 2 + 1;
  }

  void NormalizedBlockFilterNodelet::process(const cv::Mat& input_image,
                                             const std_msgs::Header& header) {
    cv::Mat filtered_image;
    cv::blur(input_image, filtered_image, cv::Size(kernel_size_, kernel_size_),
             cv::Point(-1, -1));
    pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                    filtered_image).toImageMsg());
  }

  /*** Bilateral Filter ***/
  void BilateralFilterNodelet::onInit() {
    filter_size_ = 7;
    sigma_color_ = 35;
    sigma_space_ = 5;
    SmoothingFilterNodelet::onInit();
  }

  void BilateralFilterNodelet::reconfigureCallback(
    smoothing_filter::BilateralFilterConfig& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    filter_size_ = config.filter_size;
    sigma_color_ = config.sigma_color;
    sigma_space_ = config.sigma_space;
  }

  void BilateralFilterNodelet::process(const cv::Mat& input_image,
                                       const std_msgs::Header& header) {
    cv::Mat filtered_image;
    cv::bilateralFilter(input_image, filtered_image, filter_size_, sigma_color_,
                        sigma_space_);
    pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                    filtered_image).toImageMsg());
  }

  /*** Gaussian Filter ***/
  void GaussianFilterNodelet::onInit() {
    kernel_size_ = 1;
    sigma_x_ = 10;
    sigma_y_ = 10;
    SmoothingFilterNodelet::onInit();
  }

  void GaussianFilterNodelet::reconfigureCallback(
    smoothing_filter::GaussianFilterConfig& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    kernel_size_ = config.kernel_size * 2 + 1;
    sigma_x_ = config.sigma_x;
    sigma_y_ = config.sigma_y;
  }

  void GaussianFilterNodelet::process(const cv::Mat& input_image,
                                      const std_msgs::Header& header) {
    cv::Mat filtered_image;
    cv::GaussianBlur(input_image, filtered_image,
                     cv::Size(kernel_size_, kernel_size_), sigma_x_, sigma_y_);
    pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                    filtered_image).toImageMsg());
  }

  /*** Median Filter ***/
  void MedianFilterNodelet::onInit() {
    kernel_size_ = 1;
    SmoothingFilterNodelet::onInit();
  }

  void MedianFilterNodelet::reconfigureCallback(
    smoothing_filter::MedianFilterConfig& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    kernel_size_ = config.kernel_size * 2 + 1;
  }

  void MedianFilterNodelet::process(const cv::Mat& input_image,
                                    const std_msgs::Header& header) {
    cv::Mat filtered_image;
    cv::medianBlur(input_image, filtered_image, kernel_size_);
    pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                    filtered_image).toImageMsg());
  }

  /*** Smoothing Filter ***/
  template <typename Config>
  void SmoothingFilterNodelet<Config>::filter(
    const sensor_msgs::Image::ConstPtr& input_image_msg) {
    boost::mutex::scoped_lock lock(mutex_);
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        input_image_msg, sensor_msgs::image_encodings::BGR8);
      process(cv_ptr->image, input_image_msg->header);
    } catch (cv_bridge::Exception& e) {
      NODELET_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  template <typename Config>
  void SmoothingFilterNodelet<Config>::onInit() {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(*nh_));
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    reconfigure_server_ =
      boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&SmoothingFilterNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    pub_ = advertiseImage(*pnh_, "image", 1);
    onInitPostProcess();
  }

  template <typename Config>
  void SmoothingFilterNodelet<Config>::subscribe() {
    sub_ = it_->subscribe("image", 1, &SmoothingFilterNodelet::filter, this);
  }

  template <typename Config>
  void SmoothingFilterNodelet<Config>::unsubscribe() {
    sub_.shutdown();
  }
}

#include <pluginlib/class_list_macros.h>
typedef smoothing_filter::NormalizedBlockFilterNodelet NormalizedBlockFilterNodelet;
typedef smoothing_filter::BilateralFilterNodelet BilateralFilterNodelet;
typedef smoothing_filter::GaussianFilterNodelet GaussianFilterNodelet;
typedef smoothing_filter::MedianFilterNodelet MedianFilterNodelet;
PLUGINLIB_EXPORT_CLASS(smoothing_filter::NormalizedBlockFilterNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(smoothing_filter::BilateralFilterNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(smoothing_filter::GaussianFilterNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(smoothing_filter::MedianFilterNodelet, nodelet::Nodelet);
