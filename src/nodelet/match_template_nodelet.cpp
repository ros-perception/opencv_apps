/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Ryosuke Tajima.
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
*   * Neither the name of the Ryosuke Tajima nor the names of its
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

// https://github.com/opencv/opencv/raw/master/samples/cpp/tutorial_code/Histograms_Matching/MatchTemplate_Demo.cpp
/**
 * @file MatchTemplate_Demo.cpp
 * @brief Sample code to use the function MatchTemplate
 * @author OpenCV team
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/MatchTemplateConfig.h"
#include "opencv_apps/RectArrayStamped.h"

namespace match_template
{
  class MatchTemplateNodelet:public opencv_apps::Nodelet
  {
    image_transport::Publisher img_pub_;
    image_transport::Publisher matched_img_pub_;
    image_transport::Subscriber img_sub_;
    image_transport::CameraSubscriber cam_sub_;
    ros::Publisher msg_pub_;

    boost::shared_ptr < image_transport::ImageTransport > it_;

    typedef match_template::MatchTemplateConfig Config;
    typedef dynamic_reconfigure::Server < Config > ReconfigureServer;
    Config config_;
      boost::shared_ptr < ReconfigureServer > reconfigure_server_;

    bool debug_view_;
    int match_method_;
    bool use_mask_;

      ros::Time prev_stamp_;

      cv::Mat templ_;
      cv::Mat mask_;

    void reconfigureCallback (Config & new_config, uint32_t level)
    {
      config_ = new_config;
    }

    const std::string & frameWithDefault (const std::string & frame, const std::string & image_frame)
    {
      if (frame.empty ())
        return image_frame;
      return frame;
    }

    void imageCallbackWithInfo (const sensor_msgs::ImageConstPtr & msg,
                                const sensor_msgs::CameraInfoConstPtr & cam_info)
    {
      do_work (msg, cam_info->header.frame_id);
    }

    void imageCallback (const sensor_msgs::ImageConstPtr & msg)
    {
      do_work (msg, msg->header.frame_id);
    }

    void do_work (const sensor_msgs::ImageConstPtr & msg, const std::string input_frame_from_msg)
    {
      // Work on the image.
      try
      {
        // Convert the image into something opencv can handle.
        cv::Mat frame = cv_bridge::toCvShare (msg, sensor_msgs::image_encodings::BGR8)->image;
        // Messages
        opencv_apps::RectArrayStamped rects_msg;
        rects_msg.header = msg->header;

        /// Create the result matrix
        int result_cols = frame.cols - templ_.cols + 1;
        int result_rows = frame.rows - templ_.rows + 1;
        cv::Mat result (result_rows, result_cols, CV_32FC1);

        //-- Show template
        if (debug_view_)
        {
          cv::imshow ("Template", templ_);
          int c = cv::waitKey (1);
        }

        //! [match_template]
        /// Do the Matching and Normalize
        bool method_accepts_mask = (match_method_ == CV_TM_SQDIFF || match_method_ == CV_TM_CCORR_NORMED);
        if (use_mask_ && method_accepts_mask)
        {
          matchTemplate (frame, templ_, result, match_method_, mask_);
        }
        else
        {
          matchTemplate (frame, templ_, result, match_method_);
        }
        //! [normalize]
        //normalize (result, result, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat ());
        normalize (result, result, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat ());

        //! [best_match]
        /// Localizing the best match with minMaxLoc
        double minVal;
        double maxVal;
        cv::Point minLoc;
        cv::Point maxLoc;
        cv::Point matchLoc;

        cv::minMaxLoc (result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat ());
        //! [match_loc]
        /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
        if (match_method_ == CV_TM_SQDIFF || match_method_ == CV_TM_SQDIFF_NORMED || match_method_ == CV_TM_CCORR)
        {
          matchLoc = minLoc;
        }
        else
        {
          matchLoc = maxLoc;
        }
        rectangle (frame, matchLoc, cv::Point (matchLoc.x + templ_.cols, matchLoc.y + templ_.rows), cv::Scalar::all (0),
                   2, 8, 0);
        rectangle (result, matchLoc, cv::Point (matchLoc.x + templ_.cols, matchLoc.y + templ_.rows),
                   cv::Scalar::all (0), 2, 8, 0);

        // Publish the image.
        sensor_msgs::Image::Ptr out_img =
          cv_bridge::CvImage (msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg ();
        sensor_msgs::Image::Ptr match_img =
          cv_bridge::CvImage (msg->header, sensor_msgs::image_encodings::MONO8, result).toImageMsg ();
        img_pub_.publish (out_img);
        matched_img_pub_.publish (match_img);
      }
      catch (cv::Exception & e)
      {
        NODELET_ERROR ("Image processing error: %s %s %s %i", e.err.c_str (), e.func.c_str (), e.file.c_str (), e.line);
      }

      prev_stamp_ = msg->header.stamp;
    }

    void subscribe ()
    {
      NODELET_INFO ("Subscribing to image topic.");
      if (config_.use_camera_info)
        cam_sub_ = it_->subscribeCamera ("image", 3, &MatchTemplateNodelet::imageCallbackWithInfo, this);
      else
        img_sub_ = it_->subscribe ("image", 3, &MatchTemplateNodelet::imageCallback, this);
    }

    void unsubscribe ()
    {
      NODELET_DEBUG ("Unsubscribing from image topic.");
      img_sub_.shutdown ();
      cam_sub_.shutdown ();
    }

  public:
    virtual void onInit ()
    {
      Nodelet::onInit ();
      it_ = boost::shared_ptr < image_transport::ImageTransport > (new image_transport::ImageTransport (*nh_));

      pnh_->param ("debug_view", debug_view_, false);
      pnh_->param ("match_method", match_method_, (int) CV_TM_SQDIFF);
      pnh_->param ("use_mask", use_mask_, false);
      std::string templ_file, mask_file;
      pnh_->param ("template_file", templ_file, std::string ("template.png"));
      pnh_->param ("mask_file", mask_file, std::string ("mask.png"));

      NODELET_INFO ("template_file: %s", templ_file.c_str ());

      if (debug_view_)
      {
        always_subscribe_ = true;
      }
      if (use_mask_)
      {
        mask_ = imread (mask_file, cv::IMREAD_COLOR);
      }
      if (templ_file.empty ())
      {
        NODELET_ERROR ("Cannot open template file %s", templ_file.c_str ());
        exit (0);
      }
      //templ_ = imread(templ_file, cv::IMREAD_COLOR);
      templ_ = imread (templ_file, cv::IMREAD_COLOR);

      if (debug_view_)
      {
        cv::imshow ("Match Template", templ_);
        int c = cv::waitKey (1);
      }
      prev_stamp_ = ros::Time (0, 0);

      reconfigure_server_ = boost::make_shared < dynamic_reconfigure::Server < Config > >(*pnh_);
      dynamic_reconfigure::Server < Config >::CallbackType f =
        boost::bind (&MatchTemplateNodelet::reconfigureCallback, this, _1, _2);
      reconfigure_server_->setCallback (f);

      img_pub_ = advertiseImage (*pnh_, "image", 1);
      matched_img_pub_ = advertiseImage (*pnh_, "matched_image", 1);
      msg_pub_ = advertise < opencv_apps::RectArrayStamped > (*pnh_, "matched_rectangle", 1);

      onInitPostProcess ();
    }
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (match_template::MatchTemplateNodelet, nodelet::Nodelet);
