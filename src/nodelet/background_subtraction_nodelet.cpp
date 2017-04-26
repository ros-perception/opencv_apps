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

// https://github.com/opencv/opencv/blob/2.4/samples/cpp/bgfg_segm.cpp
// https://github.com/opencv/opencv/blob/2.4/samples/cpp/bgfg_gmg.cpp
/**
 * @file bg_sub.cpp
 * @brief Background subtraction tutorial sample code
 * @author Domenico D. Bloisi
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <vector>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/BackGroundSubtractionConfig.h"

namespace background_subtraction {
class BackGroundSubtractionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef background_subtraction::BackGroundSubtractionConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  // Global Variables
  int algorithm_type_;
  // cv::Ptr<cv::BackgroundSubtractor> did not work only for GMG...
  cv::Ptr<cv::BackgroundSubtractorMOG> pMOG;  //MOG Background subtractor
  cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2;//MOG2 Background subtractor
  cv::Ptr<cv::BackgroundSubtractorGMG> pGMG;  //GMG Background subtractore
  int numInitializationFrames_;

  void reconfigureCallback(Config &new_config, uint32_t level)
  {
    config_         = new_config;
    need_config_update_ = true;
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

  void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // switch algorithm type
      if ( need_config_update_ ) {
        if ( config_.algorithm_type != algorithm_type_ ) {
          ROS_INFO("Update BackGround Subtraction Algorithm");
          algorithm_type_ = config_.algorithm_type;
          switch (algorithm_type_) {
            case background_subtraction::BackGroundSubtraction_MOG:
              if ( ! pMOG ) {
                ROS_INFO("Initialize cv::BackgroundSubtractorMOG()");
                pMOG = new cv::BackgroundSubtractorMOG();
              }
              break;
            case background_subtraction::BackGroundSubtraction_MOG2:
              if ( ! pMOG2 ) {
                ROS_INFO("Initialize cv::BackgroundSubtractorMOG2()");
                pMOG2 = new cv::BackgroundSubtractorMOG2();
              }
              break;
            case background_subtraction::BackGroundSubtraction_GMG:
              if ( ! pGMG ) {
                ROS_INFO("Initialize cv::BackgroundSubtractorGMG()");
                pGMG = new cv::BackgroundSubtractorGMG();
              }
              numInitializationFrames_ = pGMG->numInitializationFrames;
              break;
          }
        }
        switch ( algorithm_type_ ) {
          case background_subtraction::BackGroundSubtraction_MOG:
            ROS_INFO("Update cv::BackgroundSubtractorMOG()");
            pMOG = new cv::BackgroundSubtractorMOG(config_.mog_history, config_.mog_nmixtures, config_.mog_background_ratio, config_.mog_noise_sigma);
            // Length of the history
            ROS_INFO_STREAM("history         : " << config_.mog_history);
            // Number of Gaussian mixtures
            ROS_INFO_STREAM("nmixtures       : " << config_.mog_nmixtures);
            // Background ratio
            ROS_INFO_STREAM("backgroundRatio : " << config_.mog_background_ratio);
            // Noise strength
            ROS_INFO_STREAM("noiseSigma      : " << config_.mog_noise_sigma);

            // Learning rate
            ROS_INFO_STREAM("larningRate      : " << config_.mog_learning_rate);

           break;
          case background_subtraction::BackGroundSubtraction_MOG2:
            ROS_INFO("Update cv::BackgroundSubtractorMOG2()");
            pMOG2 = new cv::BackgroundSubtractorMOG2(config_.mog2_history, config_.mog2_var_threshold, config_.mog2_shadow_detection);
            // Length of the history
            ROS_INFO_STREAM("history           : " << config_.mog2_history);
            // Threshold on the squared Mahalanobis distance to decide whether it is well described by the background model (see Cthr??). This parameter does not affect the background update. A typical value could be 4 sigma, that is, varThreshold=4*4=16; (see Tb??)
            ROS_INFO_STREAM("varThreshold      : " << config_.mog2_var_threshold);
            // Parameter defining whether shadow detection should be enabled (true or false)
            ROS_INFO_STREAM("bShadowDetection : " << config_.mog2_shadow_detection);
            // Learning rate
            ROS_INFO_STREAM("larningRate      : " << config_.mog2_learning_rate);

            break;
          case background_subtraction::BackGroundSubtraction_GMG:
            ROS_INFO("Update cv::BackgroundSubtractorGMG()");

            pGMG->maxFeatures = config_.gmg_max_features;
            pGMG->learningRate = config_.gmg_learning_rate;
            pGMG->numInitializationFrames = config_.gmg_num_initialization_frames;
            pGMG->quantizationLevels = config_.gmg_quantization_levels;
            pGMG->backgroundPrior = config_.gmg_background_prior;
            pGMG->decisionThreshold = config_.gmg_decision_threshold;
            pGMG->smoothingRadius = config_.gmg_smoothing_radius;
            pGMG->updateBackgroundModel = config_.gmg_update_background_model;

            //! Total number of distinct colors to maintain in histogram.
            ROS_INFO_STREAM("maxFeatures             : " << pGMG->maxFeatures);
            //! Set between 0.0 and 1.0, determines how quickly features are "forgotten" from histograms.
            ROS_INFO_STREAM("learningRate            : " << pGMG->learningRate);
            //! Number of frames of video to use to initialize histograms.
            ROS_INFO_STREAM("numInitializationFrames : " << pGMG->numInitializationFrames);
            //! Number of discrete levels in each channel to be used in histograms.
            ROS_INFO_STREAM("quantizationLevels      : " << pGMG->quantizationLevels);
            //! Prior probability that any given pixel is a background pixel. A sensitivity parameter.
            ROS_INFO_STREAM("backgroundPrior         : " << pGMG->backgroundPrior);
            //! Value above which pixel is determined to be FG.
            ROS_INFO_STREAM("decisionThreshold       : " << pGMG->decisionThreshold);
            //! Smoothing radius, in pixels, for cleaning up FG image.
            ROS_INFO_STREAM("smoothingRadius         : " << pGMG->smoothingRadius);
            //! Perform background model update
            ROS_INFO_STREAM("updateBackgroundModel   : " << pGMG->updateBackgroundModel);
            break;
        }
        need_config_update_ = false;
      }

      cv::Mat out_image; //fg mask fg mask generated by MOG2 method
      //update the background model
      switch (algorithm_type_) {
        case background_subtraction::BackGroundSubtraction_MOG:
          pMOG->operator()(in_image, out_image);
          break;
        case background_subtraction::BackGroundSubtraction_MOG2:
          pMOG2->operator()(in_image, out_image, config_.mog2_learning_rate);
          break;
        case background_subtraction::BackGroundSubtraction_GMG:
          pGMG->operator()(in_image, out_image);
          if ( numInitializationFrames_ >= 0 ) {
            ROS_INFO_STREAM("Initializing Frames ... " << numInitializationFrames_--);
          }
          break;
      }

      //show the current frame and the fg masks
      /// Show the image
      if( debug_view_) {
        cv::imshow( window_name_, out_image );
        int c = cv::waitKey(1);
      }
      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "mono8", out_image).toImageMsg();
      img_pub_.publish(out_img);
    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", 3, &BackGroundSubtractionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &BackGroundSubtractionNodelet::imageCallback, this);
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
    prev_stamp_ = ros::Time(0, 0);

    window_name_ = "BackGroundSubtraction Demo";
    //create Background Subtractor objects
    float varThreshold = 16;
    bool bShadowDetection = true;
    algorithm_type_ = -1;

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&BackGroundSubtractionNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);

    onInitPostProcess();
  }
};
bool BackGroundSubtractionNodelet::need_config_update_ = true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(background_subtraction::BackGroundSubtractionNodelet, nodelet::Nodelet);
