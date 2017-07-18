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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include "dynamic_reconfigure/server.h"
#include "opencv_apps/nodelet.h"
#include "opencv_apps/MatchTemplateConfig.h"
#include "opencv_apps/RectArrayStamped.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

namespace match_template
{
class MatchTemplateNodelet:public opencv_apps::Nodelet
{
  image_transport::Subscriber scene_img_sub_, templ_img_sub_;
  image_transport::Publisher scene_img_pub_, templ_img_pub_, score_img_pub_;
  image_transport::CameraSubscriber scene_cam_sub_;
  ros::Publisher msg_pub_, gaze_vector_pub_;

  boost::shared_ptr < image_transport::ImageTransport > it_;

  typedef match_template::MatchTemplateConfig Config;
  typedef dynamic_reconfigure::Server < Config > ReconfigureServer;
  Config config_;
  boost::shared_ptr < ReconfigureServer > reconfigure_server_;

  image_geometry::PinholeCameraModel model_;

  bool debug_view_;
  ros::Time prev_stamp_;
  cv::Mat templ_;

  std::string scene_window_name_, templ_window_name_;
  static bool mouse_update_;
  static int mouse_event_;
  static int mouse_x_;
  static int mouse_y_;
  static bool mouse_selecting_;
  cv::Rect mouse_rect_;

  void reconfigureCallback (Config & new_config, uint32_t level)
  {
    config_ = new_config;
  }
  void imageCallbackWithInfo (const sensor_msgs::ImageConstPtr & msg,
                              const sensor_msgs::CameraInfoConstPtr & cam_info)
  {
    model_.fromCameraInfo(cam_info);
    do_work (msg, cam_info->header.frame_id);
  }

  void imageCallback (const sensor_msgs::ImageConstPtr & msg)
  {
    do_work (msg, msg->header.frame_id);
  }

  void templateCallback (const sensor_msgs::ImageConstPtr & msg)
  {
    templ_ = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8)->image;
  }

  static void onMouse( int event, int x, int y, int, void* )
  {
    mouse_update_ = true;
    mouse_event_ = event;
    mouse_x_ = x;
    mouse_y_ = y;
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

      /// Create the score matrix
      int score_cols = frame.cols - templ_.cols + 1;
      int score_rows = frame.rows - templ_.rows + 1;
      cv::Mat score (score_rows, score_cols, CV_32FC1);

      /// Debug View
      if (debug_view_)
      {
        cv::namedWindow( scene_window_name_, cv::WINDOW_AUTOSIZE );
        cv::namedWindow( templ_window_name_, cv::WINDOW_AUTOSIZE );
        cv::setMouseCallback( scene_window_name_, onMouse, 0 );
        //cv::imshow (scene_window_name_, frame);
        cv::imshow (templ_window_name_, templ_);
        int c = cv::waitKey (1);
      }
      if (mouse_update_ )
      {
        if( mouse_selecting_ )
        {
          mouse_rect_.x = MIN(mouse_x_, mouse_rect_.x);
          mouse_rect_.y = MIN(mouse_y_, mouse_rect_.y);
          mouse_rect_.width = std::abs(mouse_x_ - mouse_rect_.x);
          mouse_rect_.height = std::abs(mouse_y_ - mouse_rect_.y);
          mouse_rect_ &= cv::Rect(0, 0, frame.cols, frame.rows);
        }
        switch( mouse_event_ )
        {
          case cv::EVENT_LBUTTONDOWN:
            mouse_rect_ = cv::Rect(mouse_x_, mouse_y_, 0, 0);
            mouse_selecting_ = true;
            break;
          case cv::EVENT_LBUTTONUP:
            mouse_selecting_ = false;
            if( mouse_rect_.width > 0 && mouse_rect_.height > 0 )
            {
              // publish template image
              sensor_msgs::Image::Ptr templ_img_msg = cv_bridge::CvImage (msg->header, sensor_msgs::image_encodings::BGR8, frame(mouse_rect_)).toImageMsg ();
              templ_img_pub_.publish (templ_img_msg);
              //templ_ = frame(mouse_rect_).clone();
            }
            break;
        }
        mouse_update_ = false;
      }
      // Match template
      matchTemplate (frame, templ_, score, config_.match_method);
      /// Localizing the best match with minMaxLoc
      int remove_margin_x = templ_.cols / 2;
      int remove_margin_y = templ_.rows / 2;

      bool template_found = true;
      double minVal, maxVal;
      cv::Point minLoc, maxLoc, matchLoc;

      // Find top score position from image
      cv::minMaxLoc (score, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
      /// In some methods, lower score is better.
      if (config_.match_method == CV_TM_SQDIFF || config_.match_method == CV_TM_SQDIFF_NORMED || config_.match_method == CV_TM_CCORR)
      {
        matchLoc = minLoc;
        // Check threshold value
        if (minVal > config_.match_threshold)
        {
          template_found = false;
        }
      }
      else
      {
        matchLoc = maxLoc;
        // check threshold value
        if (maxVal < config_.match_threshold)
        {
          template_found = false;
        }
      }
      if (template_found)
      {
        rectangle (frame, matchLoc,
                   cv::Point (matchLoc.x + templ_.cols,
                              matchLoc.y + templ_.rows), cv::Scalar (0, 0, 255), 4, 8, 0);
      }
      // Publish the rectangle drawn image
      sensor_msgs::Image::Ptr match_img =
        cv_bridge::CvImage (msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg ();
      scene_img_pub_.publish (match_img);

      // Publish the monochrome score image
      normalize (score, score, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat ());
      sensor_msgs::Image::Ptr score_img =
        cv_bridge::CvImage (msg->header, sensor_msgs::image_encodings::MONO8, score).toImageMsg ();
      score_img_pub_.publish (score_img);

      if (debug_view_)
      {
        if (mouse_selecting_)
        {
          cv::Mat roi(frame, mouse_rect_);
          bitwise_not(roi, roi);
        }
        cv::imshow (scene_window_name_, frame);
      }

      // Publish the point on the image
      geometry_msgs::PointStamped point;
      point.header.stamp = ros::Time::now ();
      point.header.frame_id = input_frame_from_msg;
      point.point.x = matchLoc.x + templ_.cols / 2;
      point.point.y = matchLoc.y + templ_.rows / 2;
      point.point.z = 0;

      msg_pub_.publish (point);

      // Publish the gaze vector 
      if (config_.use_camera_info)
      {
        cv::Point center;
        center.x = matchLoc.x + templ_.cols / 2;
        center.y = matchLoc.y + templ_.rows / 2;
        cv::Point3d ray = model_.projectPixelTo3dRay (center);
        geometry_msgs::Vector3Stamped gaze_vector;
        gaze_vector.header.stamp = ros::Time::now ();
        gaze_vector.header.frame_id = input_frame_from_msg;
        gaze_vector.vector.x = ray.x;
        gaze_vector.vector.y = ray.y;
        gaze_vector.vector.z = ray.z;
        gaze_vector_pub_.publish (gaze_vector);
      }

    }
    catch (cv::Exception & e)
    {
      NODELET_ERROR ("Image processing error: %s %s %s %i", e.err.c_str (), e.func.c_str (), e.file.c_str (), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe ()
  {
    if (config_.use_camera_info)
    {
      scene_cam_sub_ = it_->subscribeCamera ("image", 3, &MatchTemplateNodelet::imageCallbackWithInfo, this);
    }
    else
    {
      scene_img_sub_ = it_->subscribe ("image", 3, &MatchTemplateNodelet::imageCallback, this);
    }
    templ_img_sub_ = it_->subscribe ("template", 3, &MatchTemplateNodelet::templateCallback, this);
  }

  void unsubscribe ()
  {
    NODELET_DEBUG ("Unsubscribing from image topic.");
    scene_img_sub_.shutdown ();
    scene_cam_sub_.shutdown ();
    templ_img_sub_.shutdown ();
  }

public:
  virtual void onInit ()
  {
    Nodelet::onInit ();
    it_ = boost::shared_ptr < image_transport::ImageTransport > (new image_transport::ImageTransport (*nh_));

    pnh_->param ("debug_view", debug_view_, true);

    templ_ = cv::Mat(1,1, CV_8UC3);
    std::string templ_file;
    pnh_->param ("template_file", templ_file, std::string (""));
    if (!templ_file.empty())
    {
      NODELET_INFO ("template_file: %s\n", templ_file.c_str());
      templ_ = imread (templ_file, cv::IMREAD_COLOR);
    }

    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    
    scene_window_name_ = "Scene Image";
    templ_window_name_ = "Template Image";

    prev_stamp_ = ros::Time (0, 0);

    reconfigure_server_ = boost::make_shared < dynamic_reconfigure::Server < Config > >(*pnh_);
    dynamic_reconfigure::Server < Config >::CallbackType f =
      boost::bind (&MatchTemplateNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback (f);

    scene_img_pub_ = advertiseImage (*pnh_, "image", 1);
    templ_img_pub_ = advertiseImage (*nh_, "template", 1);
    score_img_pub_ = advertiseImage (*pnh_, "score_image", 1);
    msg_pub_ = advertise < geometry_msgs::PointStamped > (*pnh_, "pixel_position", 1);
    gaze_vector_pub_ = advertise < geometry_msgs::Vector3Stamped > (*pnh_, "gaze_vector", 1);

    onInitPostProcess ();
  }
};
// bool MatchTemplateNodelet::need_config_update_ = false;
bool MatchTemplateNodelet::mouse_update_ = false;
int MatchTemplateNodelet::mouse_event_ = 0;
int MatchTemplateNodelet::mouse_x_ = 0;
int MatchTemplateNodelet::mouse_y_ = 0;
bool MatchTemplateNodelet::mouse_selecting_ = false;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (match_template::MatchTemplateNodelet, nodelet::Nodelet);
