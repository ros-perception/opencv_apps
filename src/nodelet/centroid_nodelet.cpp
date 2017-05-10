/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, JSK.
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
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/optflow.hpp>
#endif

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/CentroidConfig.h"
#include "opencv_apps/Point2DStamped.h"

namespace centroid {
class CentroidNodelet : public opencv_apps::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;

  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  typedef centroid::CentroidConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  bool debug_view_;

  std::string window_name_;

  void reconfigureCallback(Config &new_config, uint32_t level)
  {
    config_ = new_config;
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
      cv::Mat cvimg = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::Point2DStamped point_msg;
      point_msg.header = msg->header;


      if( debug_view_) {
        cv::namedWindow( window_name_, cv::WINDOW_AUTOSIZE );
      }

      //float start = (float)cv::getTickCount();

      // add your OpenCV process

      // set values to message
      point_msg.point.x = 1.1;
      point_msg.point.y = 2.2;

      //-- Show what you got
      if ( debug_view_) {
        cv::imshow( window_name_, cvimg );
        int c = cv::waitKey(1);
      }

      msg_pub_.publish(point_msg);
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
      cam_sub_ = it_->subscribeCamera("image", 3, &CentroidNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &CentroidNodelet::imageCallback, this);
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

    window_name_ = "centroid_debug";

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&CentroidNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    msg_pub_ = advertise<opencv_apps::Point2DStamped>(*pnh_, "centroid", 1);

    onInitPostProcess();
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(centroid::CentroidNodelet, nodelet::Nodelet);
