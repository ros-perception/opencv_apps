/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) @TODO Please fill your name here
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

// @TODO Please add link to original sample program
/**
 * This is a demo of @TODO
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// @TODO Fill more header files hHere
#include <dynamic_reconfigure/server.h>

// @TODO Note that please try to use existing opencv_apps/msg. They refrect opencv data classes.
#include "opencv_apps/Point2DStamped.h"

//
#include "opencv_apps/SampleConfig.h"
#include "opencv_apps/nodelet.h"

namespace opencv_apps
{
class SampleNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::SampleConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  bool needToInit;

  // @TODO Fill more vairables here
  static cv::Point2f point_;

  // @TODO Define callback functions as static
  static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
  {
    ROS_INFO("onMouse %d %d %d", event, x, y);  // @TODO static funcion needs ROS_INFO, but others can use NODELET_INFO
    if (event == CV_EVENT_LBUTTONDOWN)
    {
      point_ = cv::Point2f((float)x, (float)y);
    }
  }

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    // @TODO Fill your code here
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void doWork(const sensor_msgs::ImageConstPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::Point2DStamped output_msg;
      output_msg.header = msg->header;

      if (debug_view_)
      {
        // Create windows
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        // @TODO Create other GUI tools
        cv::setMouseCallback(window_name_, onMouse, 0);

        if (need_config_update_)
        {
          reconfigure_server_->updateConfig(config_);
          // @TODO Update parameters
          need_config_update_ = false;
        }
      }

      // Do the work
      if (needToInit)
      {
        // @TODO Do initial process
      }

      // @TODO Do the work
      output_msg.point.x = point_.x;
      output_msg.point.y = point_.y;

      needToInit = false;
      if (debug_view_)
      {
        cv::imshow(window_name_, image);

        char c = (char)cv::waitKey(1);
        // if( c == 27 )
        //    break;
        switch (c)
        {
          case 'r':
            needToInit = true;
            break;
        }
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, image).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(output_msg);
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
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &SampleNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &SampleNodelet::imageCallback, this);
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
      always_subscribe_ = true;
    }
    prev_stamp_ = ros::Time(0, 0);

    window_name_ = "Sample";  // @TODO Add program name
    needToInit = true;

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&SampleNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::Point2DStamped>(*pnh_, "output", 1);
    // @TODO add more advertise/services

    NODELET_INFO("Hot keys: ");
    NODELET_INFO("\tESC - quit the program");
    NODELET_INFO("\tr - auto-initialize tracking");
    // @ TODO Add more messages

    onInitPostProcess();
  }
};
bool SampleNodelet::need_config_update_ = false;
// @TODO static variable must initialized here
cv::Point2f SampleNodelet::point_ = cv::Point2f(0, 0);
}  // namespace opencv_apps

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::SampleNodelet, nodelet::Nodelet);
