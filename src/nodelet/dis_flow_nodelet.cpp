/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, Hironori Fujimoto.
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

/**
 * This is a demo of DIS optical flow algorithm
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/optflow.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/DISFlowConfig.h"
#include "opencv_apps/FlowArrayStamped.h"

namespace opencv_apps
{
/**
 * @brief A nodelet for calculate dense optical flow by DIS optical flow algorithm
 *
 * Subscribe sequential sensor_msgs/Image and publish flow between current and previous image
 * Parameter preset of quality and process speed can be selected by Dynamic Reconfigure
 */
class DISFlowNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::DISFlowConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  bool debug_view_;

  std::string window_name_;

  cv::Mat prev_gray_;

  cv::Ptr<cv::optflow::DISOpticalFlow> flow_calculator_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;

    // Create flow calculator using new parameter preset
    flow_calculator_ = cv::optflow::createOptFlow_DIS(config_.preset);
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
      // If input image has 3 channels, it is converted to grayscale by cv_bridge
      cv::Mat gray = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

      if (prev_gray_.empty())
        gray.copyTo(prev_gray_);

      if (gray.rows != prev_gray_.rows && gray.cols != prev_gray_.cols)
      {
        NODELET_WARN("Images should be of equal sizes");
        gray.copyTo(prev_gray_);
      }

      if (debug_view_)
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

      cv::Mat flow;

      // Calculate optical flow
      float start = (float)cv::getTickCount();
      flow_calculator_->calc(prev_gray_, gray, flow);
      NODELET_INFO("DISOpticalFlow::calc : %lf sec", (cv::getTickCount() - start) / cv::getTickFrequency());

      // Create and publish message
      if (msg_pub_.getNumSubscribers() > 0)
      {
        opencv_apps::FlowArrayStamped flows_msg;
        flows_msg.header = msg->header;
        flows_msg.flow.reserve(flow.total());

        for (int y = 0; y < flow.rows; ++y)
        {
          for (int x = 0; x < flow.cols; ++x)
          {
            opencv_apps::Point2D point_msg;
            point_msg.x = x;
            point_msg.y = y;

            cv::Vec2f flow_at_point = flow.at<cv::Vec2f>(y, x);
            opencv_apps::Point2D velocity_msg;
            velocity_msg.x = flow_at_point[0];
            velocity_msg.y = flow_at_point[1];

            opencv_apps::Flow flow_msg;
            flow_msg.point = point_msg;
            flow_msg.velocity = velocity_msg;
            flows_msg.flow.push_back(flow_msg);
          }
        }

        msg_pub_.publish(flows_msg);
      }

      // Debug view
      // Visualize dense optical flow as HSV color space
      if (debug_view_ || img_pub_.getNumSubscribers())
      {
        cv::Mat flow_xy[2];
        cv::split(flow, flow_xy);

        // Calculate magnitude and angle of flow
        cv::Mat flow_angle, flow_magnitude;
        cv::cartToPolar(flow_xy[0], flow_xy[1], flow_magnitude, flow_angle, true);

        cv::normalize(flow_magnitude, flow_magnitude, 0, 1, cv::NORM_MINMAX);

        cv::Mat hsv_split[3];
        // hue:        Angle of flow
        // saturation: Magnitude of flow
        // value:      Constant (1.0)
        hsv_split[0] = flow_angle / 360.0;
        hsv_split[1] = flow_magnitude;
        hsv_split[2] = cv::Mat::ones(flow.size(), CV_32F);

        cv::Mat hsv;
        cv::merge(hsv_split, 3, hsv);

        // Convert 32F to 8U
        cv::Mat hsv_8u;
        hsv.convertTo(hsv_8u, CV_8U, 255);

        // Convert HSV to BGR
        cv::Mat output_cv;
        cv::cvtColor(hsv_8u, output_cv, cv::COLOR_HSV2BGR);

        // Show OpenCV window
        if (debug_view_)
        {
          cv::imshow(window_name_, output_cv);
          cv::waitKey(1);
        }

        // Publish image
        if (img_pub_.getNumSubscribers() > 0)
        {
          sensor_msgs::Image::Ptr output_msg =
              cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, output_cv).toImageMsg();
          img_pub_.publish(output_msg);
        }
      }

      cv::swap(prev_gray_, gray);
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  }

  void subscribe() override
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", 3, &DISFlowNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &DISFlowNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  void onInit() override
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("debug_view", debug_view_, false);
    if (debug_view_)
      always_subscribe_ = true;

    window_name_ = "disflow_demo";

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&DISFlowNodelet::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<opencv_apps::FlowArrayStamped>(*pnh_, "flows", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::DISFlowNodelet, nodelet::Nodelet);
