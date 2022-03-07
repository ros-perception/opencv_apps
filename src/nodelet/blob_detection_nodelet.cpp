// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022, Hui Shi, University of Tartu.
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
*   * Neither the name of the Hui Shi, University of Tartu nor the names of its
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

// https://github.com/ros-perception/opencv_apps
/**
 * @file The package
 * @brief Opencv_apps Package
 * @author Opencv_apps team
 */

// https://github.com/spmallick/learnopencv/blob/master/BlobDetector/blob.cpp
/**
 * @file blob.cpp
 * @brief Demo code for Blob Detection
 * @author LearnOpenCV team
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/BlobDetectionConfig.h"
#include "opencv_apps/Blob.h"
#include "opencv_apps/BlobArray.h"
#include "opencv_apps/BlobArrayStamped.h"


namespace opencv_apps
{
class BlobDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Publisher thresholded_img_pub_;
  image_transport::Publisher thresholded_img_with_mask_pub_;
  image_transport::Publisher morphology_ex_img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::BlobDetectionConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_, thresholded_image_name_, thresholded_image_with_mask_name_, morphology_ex_image_name_, blob_detector_params_name_;
  static bool need_config_update_;

  cv::SimpleBlobDetector::Params params_; 
  cv::SimpleBlobDetector::Params prev_params_;
  cv::Ptr<cv::SimpleBlobDetector> detector_;

  // initial and max values of the parameters of interests.
  int hue_lower_limit_;
  int sat_lower_limit_;
  int val_lower_limit_;
  int hue_upper_limit_;
  int sat_upper_limit_;
  int val_upper_limit_;

  std::string morphology_ex_type_;
  std::string morphology_ex_type_default_value_;
  std::string prev_morphology_ex_type_;
  int morphology_ex_kernel_size_;
  int morphology_ex_kernel_size_initial_value_;

  int filter_by_color_int;  // for trackbar, trackbar requires int type.
  int blob_color_int;  // Filter by color.
  int filter_by_area_int;  // Filter by Area.
  int min_area_int;
  int max_area_int;
  int min_area_upper_limit_;
  int max_area_upper_limit_;
  int min_dist_between_blobs_int;  // min dist between blobs
  int min_dist_between_blobs_upper_limit_;
  int filter_by_circularity_int;  // Filter by Circularity
  int min_circularity_int;
  int max_circularity_int;
  int filter_by_inertia_int;  // Filter by Inertia
  int min_inertia_ratio_int;
  int max_inertia_ratio_int;
  int filter_by_convexity_int;  // Filter by Convexity
  int min_convexity_int;
  int max_convexity_int;

  // for checking if the blob detector params changes
  bool compareBlobDetectorParams(const cv::SimpleBlobDetector::Params& params_1, const cv::SimpleBlobDetector::Params& params_2)
  {
    if (params_1.filterByColor != params_2.filterByColor)
      return false;
    else if (params_1.blobColor != params_2.blobColor)
      return false;

    else if (params_1.filterByArea != params_2.filterByArea)
      return false;
    else if (params_1.minArea != params_2.minArea)
      return false;
    else if (params_1.maxArea != params_2.maxArea)
      return false;

    else if (params_1.minDistBetweenBlobs != params_2.minDistBetweenBlobs)
      return false;

    else if (params_1.filterByCircularity != params_2.filterByCircularity)
      return false;
    else if (params_1.minCircularity != params_2.minCircularity)
      return false;
    else if (params_1.maxCircularity != params_2.maxCircularity)
      return false;

    else if (params_1.filterByInertia != params_2.filterByInertia)
      return false;
    else if (params_1.minInertiaRatio != params_2.minInertiaRatio)
      return false;
    else if (params_1.maxInertiaRatio != params_2.maxInertiaRatio)
      return false;

    else if (params_1.filterByConvexity != params_2.filterByConvexity)
      return false;
    else if (params_1.minConvexity != params_2.minConvexity)
      return false;
    else if (params_1.maxConvexity != params_2.maxConvexity)
      return false;

    return true;
  }

  // ROS dynamic reconfigure call back
  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    hue_lower_limit_ = config_.hue_lower_limit;
    sat_lower_limit_ = config_.sat_lower_limit;
    val_lower_limit_ = config_.val_lower_limit;
    hue_upper_limit_ = config_.hue_upper_limit;
    sat_upper_limit_ = config_.sat_upper_limit;
    val_upper_limit_ = config_.val_upper_limit;

    morphology_ex_type_ = config_.morphology_ex_type;
    morphology_ex_kernel_size_ = config_.morphology_ex_kernel_size;

    params_.filterByColor = config_.filter_by_color;
    params_.blobColor = config_.blob_color;
    params_.filterByArea = config_.filter_by_area;
    params_.minArea = config_.min_area;
    params_.maxArea = config_.max_area;
    params_.minDistBetweenBlobs = config_.min_dist_between_blobs;
    params_.filterByCircularity = config_.filter_by_circularity;
    params_.minCircularity = config_.min_circularity;
    params_.maxCircularity = config_.max_circularity;
    params_.filterByInertia = config_.filter_by_inertia;
    params_.minInertiaRatio = config_.min_inertia_ratio;
    params_.maxInertiaRatio = config_.max_inertia_ratio;
    params_.filterByConvexity = config_.filter_by_convexity;
    params_.minConvexity = config_.min_convexity;
    params_.maxConvexity = config_.max_convexity;

    filter_by_color_int = int(params_.filterByColor);
    blob_color_int = int(params_.blobColor);
    filter_by_area_int = int(params_.filterByArea);
    min_area_int = int(params_.minArea);
    max_area_int = int(params_.maxArea);
    min_dist_between_blobs_int = int(params_.minDistBetweenBlobs);
    filter_by_circularity_int = int(params_.filterByCircularity);
    min_circularity_int = int(params_.minCircularity*100);
    max_circularity_int = int(params_.maxCircularity*100);
    filter_by_inertia_int = int(params_.filterByInertia);
    min_inertia_ratio_int = int(params_.minInertiaRatio*100);
    max_inertia_ratio_int = int(params_.maxInertiaRatio*100);
    filter_by_convexity_int = int(params_.filterByConvexity);
    min_convexity_int = int(params_.minConvexity*100);
    max_convexity_int = int(params_.maxConvexity*100);
  }

  const std::string& frameWithDefault(const std::string& frame, const std::string& image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  // opencv trackbar callback
  static void trackbarCallback(int /*unused*/, void* /*unused*/)
  {
    need_config_update_ = true;
  }

  void doWork(const sensor_msgs::ImageConstPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame_src = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::BlobArrayStamped blobs_msg;
      blobs_msg.header = msg->header;

      // Do the work
      // if the image is gray image, convert it to BGR image
      cv::Mat frame;
      if (frame_src.channels() > 1)
      {
        frame = frame_src;
      }
      else
      {
        cv::cvtColor(frame_src, frame, cv::COLOR_GRAY2BGR);
      }

      // create the windows, and attach the trackbars
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(thresholded_image_name_, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(thresholded_image_with_mask_name_, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(blob_detector_params_name_, cv::WINDOW_AUTOSIZE);

        if (morphology_ex_type_ != prev_morphology_ex_type_)
        {
          if (prev_morphology_ex_type_ == "opening" || prev_morphology_ex_type_ == "closing")
          {
            cv::destroyWindow(morphology_ex_image_name_);
          }
          morphology_ex_kernel_size_ = morphology_ex_kernel_size_initial_value_;
          config_.morphology_ex_type = morphology_ex_type_;
          config_.morphology_ex_kernel_size = morphology_ex_kernel_size_;
          reconfigure_server_->updateConfig(config_);
        }

        if (morphology_ex_type_ == "opening" || morphology_ex_type_ == "closing")
        { 
          if (morphology_ex_type_ == "opening")
          {
            morphology_ex_image_name_ = "Opening Image";
          }
          else if (morphology_ex_type_ == "closing")
          {
            morphology_ex_image_name_ = "Closing Image";
          }

          cv::namedWindow(morphology_ex_image_name_, cv::WINDOW_AUTOSIZE);
          cv::createTrackbar("MorphologyEx Kernel Size", morphology_ex_image_name_, &morphology_ex_kernel_size_, 100, trackbarCallback);
        }

        prev_morphology_ex_type_ = morphology_ex_type_;

        cv::createTrackbar("Hue Lower Limit", thresholded_image_name_, &hue_lower_limit_, 179, trackbarCallback);
        cv::createTrackbar("Hue Upper Limit", thresholded_image_name_, &hue_upper_limit_, 179, trackbarCallback);
        cv::createTrackbar("Sat Lower Limit", thresholded_image_name_, &sat_lower_limit_, 255, trackbarCallback);
        cv::createTrackbar("Sat Upper Limit", thresholded_image_name_, &sat_upper_limit_, 255, trackbarCallback);
        cv::createTrackbar("Val Lower Limit", thresholded_image_name_, &val_lower_limit_, 255, trackbarCallback);
        cv::createTrackbar("Val Upper Limit", thresholded_image_name_, &val_upper_limit_, 255, trackbarCallback);

        cv::createTrackbar("Filter By Color", blob_detector_params_name_, &filter_by_color_int, 1, trackbarCallback);
        cv::createTrackbar("Blob Color", blob_detector_params_name_, &blob_color_int, 255, trackbarCallback);
        cv::createTrackbar("Filter By Area", blob_detector_params_name_, &filter_by_area_int, 1, trackbarCallback);
        cv::createTrackbar("Min Area", blob_detector_params_name_, &min_area_int, min_area_upper_limit_, trackbarCallback);
        cv::createTrackbar("Max Area", blob_detector_params_name_, &max_area_int, max_area_upper_limit_, trackbarCallback);
        cv::createTrackbar("Min Dist Between Blobs", blob_detector_params_name_, &min_dist_between_blobs_int, min_dist_between_blobs_upper_limit_, trackbarCallback);
        cv::createTrackbar("Filter By Circularity", blob_detector_params_name_, &filter_by_circularity_int, 1, trackbarCallback);
        cv::createTrackbar("Min Circularity", blob_detector_params_name_, &min_circularity_int, 100, trackbarCallback);
        cv::createTrackbar("Max Circularity", blob_detector_params_name_, &max_circularity_int, 100, trackbarCallback);
        cv::createTrackbar("Filter By Inertia", blob_detector_params_name_, &filter_by_inertia_int, 1, trackbarCallback);
        cv::createTrackbar("Min Inertia Ratio", blob_detector_params_name_, &min_inertia_ratio_int, 100, trackbarCallback);
        cv::createTrackbar("Max Inertia Ratio", blob_detector_params_name_, &max_inertia_ratio_int, 100, trackbarCallback);
        cv::createTrackbar("Filter By Convexity", blob_detector_params_name_, &filter_by_convexity_int, 1, trackbarCallback);
        cv::createTrackbar("Min Convexity", blob_detector_params_name_, &min_convexity_int, 100, trackbarCallback);
        cv::createTrackbar("Max Convexity", blob_detector_params_name_, &max_convexity_int, 100, trackbarCallback);

        if (need_config_update_)
        {
          config_.hue_lower_limit = hue_lower_limit_;
          config_.sat_lower_limit = sat_lower_limit_;
          config_.val_lower_limit = val_lower_limit_;
          config_.hue_upper_limit = hue_upper_limit_;
          config_.sat_upper_limit = sat_upper_limit_;
          config_.val_upper_limit = val_upper_limit_;

          config_.morphology_ex_kernel_size = morphology_ex_kernel_size_;

          config_.filter_by_color = params_.filterByColor = (bool)filter_by_color_int;
          config_.blob_color = params_.blobColor = blob_color_int;
          config_.filter_by_area = params_.filterByArea = (bool)filter_by_area_int;
          config_.min_area = params_.minArea = (float)min_area_int;
          config_.max_area = params_.maxArea = (float)max_area_int;
          config_.min_dist_between_blobs = params_.minDistBetweenBlobs = (float)min_dist_between_blobs_int;
          config_.filter_by_circularity = params_.filterByCircularity = (bool)filter_by_circularity_int;
          config_.min_circularity = params_.minCircularity = (float)min_circularity_int/100;
          config_.max_circularity = params_.maxCircularity = (float)max_circularity_int/100;
          config_.filter_by_inertia = params_.filterByInertia = (bool)filter_by_inertia_int;
          config_.min_inertia_ratio = params_.minInertiaRatio = (float)min_inertia_ratio_int/100;
          config_.max_inertia_ratio = params_.maxInertiaRatio = (float)max_inertia_ratio_int/100;
          config_.filter_by_convexity = params_.filterByConvexity = (bool)filter_by_convexity_int;
          config_.min_convexity = params_.minConvexity = (float)min_convexity_int/100;
          config_.max_convexity = params_.maxConvexity = (float)max_convexity_int/100;

          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
      }

      // convert the image to HSV image
      cv::Mat HSV_image;
      cv::cvtColor(frame, HSV_image, cv::COLOR_BGR2HSV);
      // threshold the HSV image
      cv::Mat thresholded_image;
      cv::inRange(HSV_image, cv::Scalar(hue_lower_limit_, sat_lower_limit_, val_lower_limit_), cv::Scalar(hue_upper_limit_, sat_upper_limit_, val_upper_limit_), thresholded_image);
      // thresholded image with a color mask
      cv::Mat thresholded_image_with_mask;
      cv::bitwise_and(frame,frame,thresholded_image_with_mask,thresholded_image);

      // do morphological operation if required
      cv::Mat morphology_ex_image;
      if (morphology_ex_type_ == "opening" || morphology_ex_type_ == "closing")
      { 
        cv::Mat kernel; 
        // morphology_ex_kernel_size_ must be odd number
        if (morphology_ex_kernel_size_ % 2 != 1)
        {
          morphology_ex_kernel_size_ = morphology_ex_kernel_size_ + 1;
        }

        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphology_ex_kernel_size_, morphology_ex_kernel_size_), cv::Point(-1, -1));

        if (morphology_ex_type_ == "opening")
        {
          cv::morphologyEx(thresholded_image, morphology_ex_image, cv::MORPH_OPEN, kernel);
        }
        else if (morphology_ex_type_ == "closing")
        {
          cv::morphologyEx(thresholded_image, morphology_ex_image, cv::MORPH_CLOSE, kernel);
        }
      }

      // create a new blob detector if the detector params changes
      if (!compareBlobDetectorParams(params_, prev_params_))
      {
        detector_ = cv::SimpleBlobDetector::create(params_);
      }

      // keypoints will hold the results of the detection
      std::vector<cv::KeyPoint> keypoints;
      // runs the actual detection
      if (morphology_ex_type_ == "opening" || morphology_ex_type_ == "closing")
      {
        detector_->detect(morphology_ex_image, keypoints);
      }
      else
      {
        detector_->detect(thresholded_image, keypoints);
      }

      prev_params_ = params_;

      // convert the detection results to ROS messages
      for (const cv::KeyPoint& i : keypoints)
      {
        opencv_apps::Blob blob_msg;
        blob_msg.center.x = i.pt.x;
        blob_msg.center.y = i.pt.y;
        blob_msg.radius = i.size;
        blobs_msg.blobs.push_back(blob_msg);
      }

      // draw circles on the blobs detected, for displaying purposes
      cv::Mat out_image;
      drawKeypoints(frame, keypoints, out_image, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

      // shows the results
      if (debug_view_)
      {
        cv::imshow(window_name_, out_image);
        cv::imshow(thresholded_image_name_, thresholded_image);
        cv::imshow(thresholded_image_with_mask_name_, thresholded_image_with_mask);

        if (morphology_ex_type_ == "opening" || morphology_ex_type_ == "closing")
        {
          cv::imshow(morphology_ex_image_name_, morphology_ex_image);
        }

        // a waitKey for switching between different types of morphological operations via keyboard input.
        // 'o' for opening, 'c' for closing, 'n' for no morphological operation
        char c = (char)cv::waitKey(1);
        switch (c)
        {
          case 'o':
            morphology_ex_type_ = "opening";
            break;
          case 'c':
            morphology_ex_type_ = "closing";
            break;
          case 'n':
            morphology_ex_type_ = "off";
            break;
        }
      }

      // Publish the debug images if there is a subscriber.
      if (thresholded_img_pub_.getNumSubscribers() > 0)
      {
        cv::Mat out_thresholded_image;
        cv::cvtColor(thresholded_image, out_thresholded_image, cv::COLOR_GRAY2BGR);
        sensor_msgs::Image::Ptr out_thresholded_img = cv_bridge::CvImage(msg->header, "bgr8", out_thresholded_image).toImageMsg();
        thresholded_img_pub_.publish(out_thresholded_img);
      }

      if (thresholded_img_with_mask_pub_.getNumSubscribers() > 0)
      {
        sensor_msgs::Image::Ptr out_thresholded_img_with_mask = cv_bridge::CvImage(msg->header, "bgr8", thresholded_image_with_mask).toImageMsg();
        thresholded_img_with_mask_pub_.publish(out_thresholded_img_with_mask);
      }

      if (morphology_ex_img_pub_.getNumSubscribers() > 0)
      {
        if (morphology_ex_type_ == "opening" || morphology_ex_type_ == "closing")
        {
          cv::Mat out_morphology_ex_image;
          cv::cvtColor(morphology_ex_image, out_morphology_ex_image, cv::COLOR_GRAY2BGR);
          sensor_msgs::Image::Ptr out_morphology_ex_img = cv_bridge::CvImage(msg->header, "bgr8", out_morphology_ex_image).toImageMsg();
          morphology_ex_img_pub_.publish(out_morphology_ex_img);
        }
      }
      // convert the output image to ROS messages
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      // Publish the output image and blobs messages.
      img_pub_.publish(out_img);
      msg_pub_.publish(blobs_msg);
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &BlobDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &BlobDetectionNodelet::imageCallback, this);
  }

  void unsubscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    // initialize the nodelet
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("debug_view", debug_view_, false);
    pnh_->param("morphology_ex_type", morphology_ex_type_, morphology_ex_type_default_value_);
    pnh_->param("min_area_upper_limit", min_area_upper_limit_, 40000);
    pnh_->param("max_area_upper_limit", max_area_upper_limit_, 40000);
    pnh_->param("min_dist_between_blobs_upper_limit", min_dist_between_blobs_upper_limit_, 200);

    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = ros::Time(0, 0);

    window_name_ = "Blob Detection Demo";
    thresholded_image_name_ = "Thresholded Image";
    thresholded_image_with_mask_name_ = "Thresholded Image With Mask";
    morphology_ex_image_name_ = "Opening Image";
    blob_detector_params_name_ = "Blob Detector Params";

    morphology_ex_type_default_value_ = "off";
    prev_morphology_ex_type_ = "off";
    morphology_ex_kernel_size_initial_value_ = 3;

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&BlobDetectionNodelet::reconfigureCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::BlobArrayStamped>(*pnh_, "blobs", 1);

    thresholded_img_pub_ = advertiseImage(*pnh_, "thresholded_image", 1);
    thresholded_img_with_mask_pub_ = advertiseImage(*pnh_, "thresholded_image_with_mask", 1);
    morphology_ex_img_pub_ = advertiseImage(*pnh_, "morphology_ex_image", 1);

    onInitPostProcess();
  }
};
bool BlobDetectionNodelet::need_config_update_ = false;
}  // namespace opencv_apps

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::BlobDetectionNodelet, nodelet::Nodelet);
