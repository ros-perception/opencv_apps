// -*- coding:utf-8-unix; mode: c++; indent-tabs-mode: nil; c-basic-offset: 2; -*-
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughCircle_Demo.cpp
/**
 * @file HoughCircle_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
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

// 1. opencv trackbar debug
//2. ros dynamic  setup mode, launch file with dynamic and rviz with img topic attached.
//3. nothing, only launch file load the yaml file loading,    2 and 3 use the same launch file, arg-mode, default-simple run which is the 3rd case
// another mode is 1st, then 2nd.
// Q1. setup mode, there is no thresholded image publish in the node.
// Q2. in launch file, how to switch between the launch of different group of nodes.
namespace opencv_apps
{
class BlobDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
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

  std::string window_name_, thresholded_image_name_, thresholded_image_with_mask_name_, blob_detector_params_name_, opening_image_name_, closing_image_name_; // reference to camshift_nodelet
  static bool need_config_update_;

  cv::SimpleBlobDetector::Params params_; //
  cv::SimpleBlobDetector::Params prev_params_;

    //this is for opencv version 2: SimpleBlobDetector detector(params); check the following link for more info
  // https://learnopencv.com/blob-detection-using-opencv-python-c/
  //cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params); // this line was without the 3 cv::
  // where should we create the detector and params ? https://ip.festo-didactic.com/DigitalEducation/EITManufacturing/UNICO/FDRenderer/index.html?LearningPath=0a4e3fe94887400cad125a990fd2d518&Language=EN&FDEP=true&NoPHP=true
  cv::Ptr<cv::SimpleBlobDetector> detector;

  // initial and max values of the parameters of interests.
  int lowHue;
  int lowSat;
  int lowVal;
  int highHue;  // h_limit_max or high_hue_  ?
  int highSat;
  int highVal;

  std::string morphology_ex_type_;
  int morphology_ex_kernel_size_;
  int morphology_ex_kernel_size_initial_value_;

  int filterByColor_int; // for trackbar, because trackbar only accept int type.
  int blobColor_int;

  int filterByArea_int; // ???
  int minArea_int;
  int maxArea_int;
  int max_minArea_;
  int max_maxArea_;

  int minDistBetweenBlobs_int;
  int max_minDistBetweenBlobs_;
    // Filter by Area.

  //   // Filter by Circularity
  int filterByCircularity_int;
  int minCircularity_int;
  int maxCircularity_int;

  // Filter by Inertia
  int filterByInertia_int;
  int minInertiaRatio_int;
  int maxInertiaRatio_int;

   // Filter by Convexity
  int filterByConvexity_int;
  int minConvexity_int;
  int maxConvexity_int;

  bool compareBlobDetectorParams(cv::SimpleBlobDetector::Params params_1, cv::SimpleBlobDetector::Params params_2)// contour_moments line 61 counter1 counter2, &
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


  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;

    lowHue = config_.lowHue;
    lowSat = config_.lowSat;
    lowVal = config_.lowVal;
    highHue = config_.highHue;
    highSat = config_.highSat;
    highVal = config_.highVal;

    morphology_ex_kernel_size_ = config_.morphology_ex_kernel_size;

    params_.filterByColor = config_.filterByColor;
    params_.blobColor = config_.blobColor;

    //   // Filter by Area.
    params_.filterByArea = config_.filterByArea;
    params_.minArea = config_.minArea;
    params_.maxArea = config_.maxArea;

    params_.minDistBetweenBlobs = config_.minDistBetweenBlobs;

    //   // Filter by Circularity
    params_.filterByCircularity = config_.filterByCircularity;
    params_.minCircularity = config_.minCircularity;
    params_.maxCircularity = config_.maxCircularity;

      // Filter by Inertia
    params_.filterByInertia = config_.filterByInertia;
    params_.minInertiaRatio = config_.minInertiaRatio;
    params_.maxInertiaRatio = config_.maxInertiaRatio;
      // Filter by Convexity
    params_.filterByConvexity = config_.filterByConvexity;
    params_.minConvexity = config_.minConvexity;
    params_.maxConvexity = config_.maxConvexity;


    filterByColor_int = int(params_.filterByColor);
    blobColor_int = int(params_.blobColor);

    filterByArea_int = int(params_.filterByArea);
    minArea_int = int(params_.minArea);
    maxArea_int = int(params_.maxArea);

    minDistBetweenBlobs_int = int(params_.minDistBetweenBlobs);

    filterByCircularity_int = int(params_.filterByCircularity);
    minCircularity_int = int(params_.minCircularity*100);
    std::cout << "test reconfigure callback convert int(params_.minCircularity*100) to : " << minCircularity_int << std::endl;
    maxCircularity_int = int(params_.maxCircularity*100);
    std::cout << "test reconfigure callback convert int(params_.maxCircularity*100) to : " << maxCircularity_int << std::endl;

    filterByInertia_int = int(params_.filterByInertia);
    minInertiaRatio_int = int(params_.minInertiaRatio*100);
    std::cout << "test reconfigure callback convert int(params_.minInertiaRatio*100) to : " << minInertiaRatio_int << std::endl;
    maxInertiaRatio_int = int(params_.maxInertiaRatio*100);
    std::cout << "test reconfigure callback convert int(params_.maxInertiaRatio*100) to : " << maxInertiaRatio_int << std::endl;

    filterByConvexity_int = int(params_.filterByConvexity);
    minConvexity_int = int(params_.minConvexity*100);
    std::cout << "test reconfigure callback convert int(params_.minConvexity*100) to : " << minConvexity_int << std::endl;
    maxConvexity_int = int(params_.maxConvexity*100);
    std::cout << "test reconfigure callback convert int(params_.maxConvexity*100) to : " << maxConvexity_int << std::endl;

    ROS_INFO("test for reconfigure call back.");
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

  //  static void trackbarCallback(int /*unused*/, void* /*unused*/)
  static void trackbarCallback(int value, void* userdata)
  {
    need_config_update_ = true;
    ROS_INFO("test for trackbar call back.");
  }

  void doWork(const sensor_msgs::ImageConstPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::BlobArrayStamped blobs_msg;
      blobs_msg.header = msg->header;

      // Do the work
  // do we need to check whether the frame is BGR ? since we convert it from BGR to HSV
// Serena : change to ours, do thresholding here
      // if (frame.channels() > 1)
      // {
      //   cv::cvtColor(frame, src_gray, cv::COLOR_BGR2GRAY);
      // }
      // else
      // {
      //   src_gray = frame;
      // }

      // create the main window, and attach the trackbars
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(thresholded_image_name_, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(thresholded_image_with_mask_name_, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(blob_detector_params_name_, cv::WINDOW_AUTOSIZE);
        
        if (morphology_ex_type_ == "opening")
        {
          cv::namedWindow(opening_image_name_, cv::WINDOW_AUTOSIZE);
          cv::createTrackbar("morphology_ex_kernel_size", opening_image_name_, &morphology_ex_kernel_size_, 100, trackbarCallback);
        }

        else if (morphology_ex_type_ == "closing")
        {
          cv::namedWindow(closing_image_name_, cv::WINDOW_AUTOSIZE);
          cv::createTrackbar("morphology_ex_kernel_size", closing_image_name_, &morphology_ex_kernel_size_, 100, trackbarCallback);
        }

        cv::createTrackbar("lowHue", thresholded_image_name_, &lowHue, 179, trackbarCallback);// should we increase the range? const int max_value_H = 360/2; https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
        cv::createTrackbar("lowSat", thresholded_image_name_, &lowSat, 255, trackbarCallback);
        cv::createTrackbar("lowVal", thresholded_image_name_, &lowVal, 255, trackbarCallback);

        cv::createTrackbar("highHue", thresholded_image_name_, &highHue, 179, trackbarCallback);
        cv::createTrackbar("highSat", thresholded_image_name_, &highSat, 255, trackbarCallback);
        cv::createTrackbar("highVal", thresholded_image_name_, &highVal, 255, trackbarCallback);

        cv::createTrackbar("filterByColor", blob_detector_params_name_, &filterByColor_int, 1, trackbarCallback);
        cv::createTrackbar("blobColor", blob_detector_params_name_, &blobColor_int, 255, trackbarCallback);

        cv::createTrackbar("filterByArea", blob_detector_params_name_, &filterByArea_int, 1, trackbarCallback);
        cv::createTrackbar("minArea", blob_detector_params_name_, &minArea_int, max_minArea_, trackbarCallback);
        cv::createTrackbar("maxArea", blob_detector_params_name_, &maxArea_int, max_maxArea_, trackbarCallback);//  ??
        
        cv::createTrackbar("minDistBetweenBlobs", blob_detector_params_name_, &minDistBetweenBlobs_int, max_minDistBetweenBlobs_, trackbarCallback);// ??
        
        cv::createTrackbar("filterByCircularity", blob_detector_params_name_, &filterByCircularity_int, 1, trackbarCallback);
        cv::createTrackbar("minCircularity", blob_detector_params_name_, &minCircularity_int, 100, trackbarCallback);
        cv::createTrackbar("maxCircularity", blob_detector_params_name_, &maxCircularity_int, 100, trackbarCallback);

        cv::createTrackbar("filterByInertia", blob_detector_params_name_, &filterByInertia_int, 1, trackbarCallback);
        cv::createTrackbar("minInertiaRatio", blob_detector_params_name_, &minInertiaRatio_int, 100, trackbarCallback);
        cv::createTrackbar("maxInertiaRatio", blob_detector_params_name_, &maxInertiaRatio_int, 100, trackbarCallback);

        cv::createTrackbar("filterByConvexity", blob_detector_params_name_, &filterByConvexity_int, 1, trackbarCallback);
        cv::createTrackbar("minConvexity", blob_detector_params_name_, &minConvexity_int, 100, trackbarCallback);
        cv::createTrackbar("maxConvexity", blob_detector_params_name_, &maxConvexity_int, 100, trackbarCallback);

        if (need_config_update_)
        {
          config_.lowHue = lowHue;
          config_.lowSat = lowSat;
          config_.lowVal = lowVal;
          config_.highHue = highHue;
          config_.highSat = highSat;
          config_.highVal = highVal;

          config_.morphology_ex_kernel_size = morphology_ex_kernel_size_;


          config_.filterByColor = params_.filterByColor = (bool)filterByColor_int;
          config_.blobColor = params_.blobColor = blobColor_int;

          // // Filter by Area.
          config_.filterByArea = params_.filterByArea = (bool)filterByArea_int;
          config_.minArea = params_.minArea = (double)minArea_int;//should be the same after printing the 2 types
          std::cout << "test trackbar changes params_.minArea to : " << params_.minArea << std::endl;
          config_.maxArea = params_.maxArea = (double)maxArea_int;// use float
          std::cout << "test trackbar changes params_.maxArea to : " << params_.maxArea << std::endl;

          config_.minDistBetweenBlobs = params_.minDistBetweenBlobs = (double)minDistBetweenBlobs_int;
          std::cout << "test trackbar changes params_.minDistBetweenBlobs to : " << params_.minDistBetweenBlobs << std::endl;


          // // Filter by Circularity
          config_.filterByCircularity = params_.filterByCircularity = (bool)filterByCircularity_int;
          config_.minCircularity = params_.minCircularity = (double)minCircularity_int/100.00;
          std::cout << "test trackbar changes params_.minCircularity to : " << params_.minCircularity << std::endl;
          config_.maxCircularity = params_.maxCircularity = (double)maxCircularity_int/100.00;
          std::cout << "test trackbar changes params_.maxCircularity to : " << params_.maxCircularity << std::endl;
          
          // Filter by Inertia
          config_.filterByInertia = params_.filterByInertia = (bool)filterByInertia_int;
          config_.minInertiaRatio = params_.minInertiaRatio = (double)minInertiaRatio_int/100.00;
          std::cout << "test trackbar changes params_.minInertiaRatio to : " << params_.minInertiaRatio << std::endl;
          config_.maxInertiaRatio = params_.maxInertiaRatio = (double)maxInertiaRatio_int/100.00;
          std::cout << "test trackbar changes params_.maxInertiaRatio to : " << params_.maxInertiaRatio << std::endl;          
         
          // Filter by Convexity
          config_.filterByConvexity = params_.filterByConvexity = (bool)filterByConvexity_int;
          config_.minConvexity = params_.minConvexity = (double)minConvexity_int/100.00;
          std::cout << "test trackbar changes params_.minConvexity to : " << params_.minConvexity << std::endl;
          config_.maxConvexity = params_.maxConvexity = (double)maxConvexity_int/100.00;
          std::cout << "test trackbar changes params_.maxConvexity to : " << params_.maxConvexity << std::endl;
          
          ROS_INFO("test for before updating config. when the trackbar moves, the params server is updated.");
          reconfigure_server_->updateConfig(config_);
          need_config_update_ = false;
        }
      }

      cv::Mat HSV_image;
      cv::cvtColor(frame, HSV_image, cv::COLOR_BGR2HSV); // hsv hue range is 180 and hsv full is 360
      cv::Mat thresholded_image;
      cv::inRange(HSV_image, cv::Scalar(lowHue, lowSat, lowVal), cv::Scalar(highHue, highSat, highVal), thresholded_image); // there were not the 2 cv::

      cv::Mat thresholded_image_with_mask;
      cv::bitwise_and(frame,frame,thresholded_image_with_mask,thresholded_image); //bitwise_and(img,mask,m_out);
 //Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5), Point(-1, -1));

 //Mat element = getStructuringElement(MORPH_RECT,
 //Size(2 * morph_size + 1,2 * morph_size + 1),
        //Point(morph_size, morph_size));

      cv::Mat kernel; 
      cv::Mat opening_image;
      cv::Mat closing_image;

      if (morphology_ex_type_ == "opening")
      {
        if (morphology_ex_kernel_size_ % 2 != 1)
        {
          morphology_ex_kernel_size_ = morphology_ex_kernel_size_ + 1;
        }

        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphology_ex_kernel_size_, morphology_ex_kernel_size_), cv::Point(-1, -1));


        cv::morphologyEx(thresholded_image, opening_image, cv::MORPH_OPEN, kernel);
      //morphologyEx(img, open, CV_MOP_OPEN, kernel);

          //morphologyEx( src, dst, MORPH_TOPHAT, element, Point(-1,-1), i );   
    //morphologyEx( src, dst, MORPH_TOPHAT, element ); // here iteration=1
//MORPH_OPEN
      }

      else if (morphology_ex_type_ == "closing")
      {
        if (morphology_ex_kernel_size_ % 2 != 1)
        {
          morphology_ex_kernel_size_ = morphology_ex_kernel_size_ + 1;
        }

        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphology_ex_kernel_size_, morphology_ex_kernel_size_), cv::Point(-1, -1));

      //morphologyEx(img, close, CV_MOP_CLOSE, kernel);
      //morphologyEx(image, output, MORPH_CLOSE, element,Point(-1, -1), 2);
        cv::morphologyEx(thresholded_image, closing_image, cv::MORPH_CLOSE, kernel);
      }

     // Serena : not related to us, or later we use the same way to check on something if needed
      // those paramaters cannot be =0
      // so we must check here
      //canny_threshold_ = std::max(canny_threshold_, 1.0);
      //accumulator_threshold_ = std::max(accumulator_threshold_, 1.0);


      if (!compareBlobDetectorParams(params_, prev_params_))
      {
        detector = cv::SimpleBlobDetector::create(params_);
        ROS_INFO("test for create new detector.");
      }

      // runs the detection, and update the display
      // will hold the results of the detection
      std::vector<cv::KeyPoint> keypoints;
      // runs the actual detection
      if (morphology_ex_type_ == "opening")
      {
        detector->detect(opening_image, keypoints);
      }

      else if (morphology_ex_type_ == "closing")
      {
        detector->detect(closing_image, keypoints);
      }

      else
      {
        detector->detect(thresholded_image, keypoints);
      }

      prev_params_ = params_;

      if (keypoints.size() > 0)
      {
        std::cout << "test if there is keypoint detected : " << keypoints[0].size << std::endl;
      }



      // cv::Mat out_image;
      // will we encounter this situation ?? if (frame.channels() == 1)
      // {
      //   cv::cvtColor(frame, out_image, cv::COLOR_GRAY2BGR);
      // }
      // else
      // {
      //   out_image = frame;
      // }

      // clone the colour, input image for displaying purposes

      for (const cv::KeyPoint& i : keypoints)
      {
        // cv::Point center(cvRound(i.pt[0]), cvRound(i.pt[1])); // should we round or not ?
        // int radius = cvRound(i.size); // does not need to re-define.
// Serena : change to ours, generate the blob messages here
        opencv_apps::Blob blob_msg;
        // blob_msg.center.x = center.x;
        // blob_msg.center.y = center.y;
        // blob_msg.radius = radius;
        blob_msg.center.x = i.pt.x;
        blob_msg.center.y = i.pt.y;
        blob_msg.radius = i.size;
        blobs_msg.blobs.push_back(blob_msg); // why do not put in blobs_msg directly ?
      }

      cv::Mat out_image;
      drawKeypoints(frame, keypoints, out_image, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS ); // there were not the 2 cv::

      // shows the results

      if (debug_view_)
      {
        cv::imshow(window_name_, out_image);// show debug image or out image?
        cv::imshow(thresholded_image_name_, thresholded_image);
        cv::imshow(thresholded_image_with_mask_name_, thresholded_image_with_mask);

        if (morphology_ex_type_ == "opening")
        {
          cv::imshow(opening_image_name_, opening_image);
        }

        else if (morphology_ex_type_ == "closing")
        {
          cv::imshow(closing_image_name_, closing_image);
        }
        
        char c = (char)cv::waitKey(1);

        // if( c == 27 )
        //    break;
        switch (c)
        {
          case 'o':
            morphology_ex_type_ = "opening";
            cv::destroyWindow(closing_image_name_);
            morphology_ex_kernel_size_ = morphology_ex_kernel_size_initial_value_;
            break;
          case 'c':
            morphology_ex_type_ = "closing";
            cv::destroyWindow(opening_image_name_);
            morphology_ex_kernel_size_ = morphology_ex_kernel_size_initial_value_;
            break;
          case 'n':
            morphology_ex_type_ = "off";
            cv::destroyWindow(opening_image_name_);
            cv::destroyWindow(closing_image_name_);
            morphology_ex_kernel_size_ = morphology_ex_kernel_size_initial_value_;
            break;
        }
      }

      // Publish the image.
      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(blobs_msg); //s
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
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("debug_view", debug_view_, false);

    if (debug_view_)
    {
      always_subscribe_ = debug_view_; // or true
    }
    prev_stamp_ = ros::Time(0, 0);

    window_name_ = "Blob Detection Demo";
    thresholded_image_name_ = "Thresholded Image";
    thresholded_image_with_mask_name_ = "Thresholded Image With Mask";
    opening_image_name_ = "Opening Image";
    closing_image_name_ = "Closing Image";
    blob_detector_params_name_ = "Blob Detector Params";
    // delete output screen in launch file.
    ROS_INFO("test for oninit.");
    
    morphology_ex_kernel_size_initial_value_ = 3;
    max_minArea_ = 10000; // if 2560*1600 = 4096000
    max_maxArea_ = 10000; // if 2560*1600 = 4096000
    max_minDistBetweenBlobs_ = 1000000; // if 2560*1600 = 4096000

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&BlobDetectionNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::BlobArrayStamped>(*pnh_, "blobs", 1);

    onInitPostProcess();
  }
};
bool BlobDetectionNodelet::need_config_update_ = false;
}  // namespace opencv_apps

// the following block (namespace) is not needed in our case
namespace blob_detection
{
class BlobDetectionNodelet : public opencv_apps::BlobDetectionNodelet
{
public:
  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    //ROS_WARN("DeprecationWarning: Nodelet hough_circles/hough_circles is deprecated, "
             //"and renamed to opencv_apps/hough_circles.");
    opencv_apps::BlobDetectionNodelet::onInit();
  }
};
}  // namespace hough_circles

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::BlobDetectionNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(blob_detection::BlobDetectionNodelet, nodelet::Nodelet);
