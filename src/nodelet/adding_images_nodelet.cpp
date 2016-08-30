// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) JSK, 2016 Lab
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

// https://github.com/opencv/opencv/tree/2.4/samples/cpp/tutorial_code/ImgProc/AddingImages.cpp
/**
 * This is a demo of adding image (linear blending).
 */

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "opencv_apps/AddingImagesConfig.h"
#include "opencv_apps/nodelet.h"

namespace adding_images {
  class AddingImagesNodelet : public opencv_apps::Nodelet {
  private:
    boost::shared_ptr<image_transport::ImageTransport> it_;

    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image> ApproxSyncPolicy;

    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    typedef adding_images::AddingImagesConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    Config config_;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;

    image_transport::SubscriberFilter sub_image1_, sub_image2_;
    image_transport::Publisher img_pub_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy> > async_;
    boost::mutex mutex_;

    bool approximate_sync_;
    double alpha_;
    double beta_;
    double gamma_;

    void subscribe() {
      NODELET_DEBUG("Subscribing to image topic.");
      sub_image1_.subscribe(*it_, "image1", 1);
      sub_image2_.subscribe(*it_, "image2", 1);
      if (approximate_sync_) {
        async_ =
          boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(
            100);
        async_->connectInput(sub_image1_, sub_image2_);
        async_->registerCallback(
          boost::bind(&AddingImagesNodelet::add, this, _1, _2));
      } else {
        sync_ =
          boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        sync_->connectInput(sub_image1_, sub_image2_);
        sync_->registerCallback(
          boost::bind(&AddingImagesNodelet::add, this, _1, _2));
      }
    }

    void unsubscribe() {
      NODELET_DEBUG("Unsubscribing from image topic.");
      sub_image1_.unsubscribe();
      sub_image2_.unsubscribe();
    }

    void reconfigureCallback(Config& config, uint32_t level) {
      boost::mutex::scoped_lock lock(mutex_);
      config_ = config;
      alpha_ = config.alpha;
      beta_ = 1.0 - alpha_;
      gamma_ = config.gamma;
    }

    void add(const sensor_msgs::Image::ConstPtr& image_msg1,
             const sensor_msgs::Image::ConstPtr& image_msg2) {
      cv::Mat image1 =
        cv_bridge::toCvShare(image_msg1, image_msg1->encoding)->image;
      cv::Mat image2 =
        cv_bridge::toCvShare(image_msg2, image_msg2->encoding)->image;
      cv::Mat result_image;
      cv::addWeighted(image1, alpha_, image2, beta_, gamma_, result_image);
      img_pub_.publish(cv_bridge::CvImage(image_msg1->header,
                                          image_msg1->encoding,
                                          result_image).toImageMsg());
    }

  public:
    virtual void onInit() {
      Nodelet::onInit();
      it_ = boost::shared_ptr<image_transport::ImageTransport>(
        new image_transport::ImageTransport(*nh_));

      ////////////////////////////////////////////////////////
      // Dynamic Reconfigure
      ////////////////////////////////////////////////////////
      reconfigure_server_ =
        boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&AddingImagesNodelet::reconfigureCallback, this, _1, _2);
      reconfigure_server_->setCallback(f);

      pnh_->param("approximate_sync", approximate_sync_, true);
      img_pub_ = advertiseImage(*pnh_, "image", 1);
      onInitPostProcess();
    }
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(adding_images::AddingImagesNodelet, nodelet::Nodelet);
