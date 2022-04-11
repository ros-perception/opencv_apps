/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022, Yoshiki Obinata.
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

#include <opencv2/saliency.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/ObjectnessConfig.h"
#include "opencv_apps/RectArrayStamped.h"

namespace opencv_apps
{
class ObjectnessNodelet : public opencv_apps::Nodelet
{
  std::string window_name_;

  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  std::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::ObjectnessConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  Config config_;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;

  boost::mutex mutex_;

  cv::Ptr<cv::saliency::Saliency> objectnessAlgorithm;

  int nss_;
  std::string training_path_;
  int max_objectness_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    nss_ = config_.nss;
    max_objectness_ = config_.max_objectness;
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
    try
    {
      // declaration
      std::vector<cv::Vec4i> objectnessBoxes_;
      cv::Mat frame_, debug_frame_;
      opencv_apps::RectArrayStamped rects_;

      // convert the image msg to cv object
      if (msg->encoding == sensor_msgs::image_encodings::BGR8)
      {
        frame_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      }
      else if (msg->encoding == sensor_msgs::image_encodings::RGB8)
      {
        frame_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;
        cv::cvtColor(frame_, frame_, CV_RGB2BGR);
      }
      else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
      {
        frame_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
      }
      else
      {
        NODELET_ERROR_STREAM("Not supported image encoding: " << msg->encoding);
      }

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        debug_frame_ = frame_.clone();
      }

      // reconfigure
      objectnessAlgorithm.dynamicCast<cv::saliency::ObjectnessBING>()->setNSS(nss_);

      // detect
      if (objectnessAlgorithm->computeSaliency(frame_, objectnessBoxes_))
      {
        rects_.header.frame_id = input_frame_from_msg;  // set header
        // for(const cv::Vec4i& b : objectnessBoxes_){
        for (int i = 0; i < std::min((int)objectnessBoxes_.size(), max_objectness_); i++)
        {
          // array b has (minX, minY, maxX, maxY)
          cv::Vec4i& b = objectnessBoxes_.at(i);
          opencv_apps::Rect rect_;
          rect_.x = float(b[0] + b[2]) / 2.0;
          rect_.y = float(b[1] + b[3]) / 2.0;
          rect_.width = b[2] - b[0];
          rect_.height = b[3] - b[1];
          rects_.rects.push_back(rect_);
          // draw rect in debug view
          if (debug_view_)
            cv::rectangle(debug_frame_, cv::Vec2i(b[0], b[1]), cv::Vec2i(b[2], b[3]), cv::Vec3i(0, 0, 255), 3);
        }
        // publish
        msg_pub_.publish(rects_);
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, debug_frame_);
        int c = cv::waitKey(1);
      }
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i\n", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
      NODELET_ERROR("Please check whether you set the training path correctly at the same time\n");
    }
  }

  void subscribe()
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &ObjectnessNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &ObjectnessNodelet::imageCallback, this);
  }

  void unsubscribe()
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  virtual void onInit()
  {
    Nodelet::onInit();
    it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));
    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("debug_view", debug_view_, false);
    pnh_->getParam("training_path", training_path_);

    window_name_ = "Objectness View";

    objectnessAlgorithm = cv::saliency::ObjectnessBING::create();  // support BING

    NODELET_INFO_STREAM("BING Training path: " << training_path_);
    objectnessAlgorithm.dynamicCast<cv::saliency::ObjectnessBING>()->setTrainingPath(training_path_);

    reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
        std::bind(&ObjectnessNodelet::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);
    reconfigure_server_->setCallback(f);

    msg_pub_ = advertise<opencv_apps::RectArrayStamped>(*pnh_, "rect", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::ObjectnessNodelet, nodelet::Nodelet);
