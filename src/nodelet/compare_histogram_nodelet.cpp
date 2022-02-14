/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Gaël Écorchard, Czech Technical University.
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
*   * Neither the name of the Gaël Écorchard nor the names of its
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
 * Compute the histogram of intensities and publish it as an image.
 */

// https://github.com/opencv/opencv/tree/3.4/samples/cpp/tutorial_code/Histograms_Matching/calcHist_Demo.cpp
/**
 * @function calcHist_Demo.cpp
 * @brief Demo code to use the function calcHist
 * @author OpenCV team
 */

// https://github.com/opencv/opencv/blob/3.4/samples/cpp/tutorial_code/Histograms_Matching/compareHist_Demo.cpp
/**
 * @file compareHist_Demo.cpp
 * @brief Sample code to use the function compareHist
 * @author OpenCV team
 */

#include <ros/ros.h>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>

#include "opencv_apps/CompareHistogramConfig.h"

namespace opencv_apps
{
class CompareHistogramNodelet : public opencv_apps::Nodelet
{
  std::string window_name_;
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_, distance_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  typedef opencv_apps::CompareHistogramConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  Config config_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  int queue_size_;
  bool debug_view_;
  int histogram_size_;
  bool uniform_;
  bool accumulate_;
  std::vector<cv::Mat> reference_histogram_;

  void reconfigureCallback(Config& new_config, uint32_t level)
  {
    config_ = new_config;
    histogram_size_ = config_.histogram_size;
    uniform_ = config_.uniform;
    accumulate_ = config_.accumulate;
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
      // Convert the image into something opencv can handle.
      cv::Mat frame;
      if (msg->encoding == sensor_msgs::image_encodings::BGR8)
        frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
        frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      //  Draw contours
      cv::Mat drawing;
      bool publish_drawing = (img_pub_.getNumSubscribers() > 0);

      //  Separate the image
      std::vector<cv::Mat> planes;
      cv::split(frame, planes);

      //  Set the range
      float range[] = { 0, 256 };  // the upper boundary is exclusive
      const float* hist_range[] = { range };

      // Compute the histograms
      std::vector<cv::Mat> hist;
      hist.resize(planes.size());
      for (unsigned int j = 0; j < hist.size(); j++)
      {
        cv::calcHist(&planes[j], 1, nullptr, cv::Mat(), hist[j], 1, &histogram_size_, hist_range, uniform_, accumulate_);
      }

      // Publishe the results
      std_msgs::Float64MultiArray hist_msg;
      hist_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      hist_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      hist_msg.layout.dim[0].size = hist.size();
      hist_msg.layout.dim[0].stride = hist.size() * histogram_size_;
      hist_msg.layout.dim[1].size = histogram_size_;
      hist_msg.layout.dim[1].stride = histogram_size_;
      for (auto& j : hist)
      {
        cv::normalize(j, j, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
        for (unsigned int i = 0; i < histogram_size_; i++)
        {
          hist_msg.data.push_back(j.at<float>(i));
        }
      }
      msg_pub_.publish(hist_msg);

      // Apply the histogram comparison methods
      if (!reference_histogram_.empty())
      {
        int compare_method;
        switch (config_.histogram_comparison_type)
        {
          case opencv_apps::CompareHistogram_Correlation:
            compare_method = cv::HISTCMP_CORREL;
            break;
          case opencv_apps::CompareHistogram_Chi_Square:
            compare_method = cv::HISTCMP_CHISQR;
            break;
          case opencv_apps::CompareHistogram_Intersection:
            compare_method = cv::HISTCMP_INTERSECT;
            break;
          case opencv_apps::CompareHistogram_Bhattacharyya:
            compare_method = cv::HISTCMP_BHATTACHARYYA;
            break;
        }
        double distance = 1;
        for (unsigned int j = 0; j < hist.size(); j++)
        {
          distance = distance * cv::compareHist(reference_histogram_[j], hist[j], compare_method);
        }
        std_msgs::Float64 distance_msg;
        distance_msg.data = distance;
        distance_pub_.publish(distance_msg);
      }

      // Draw the histograms
      if (debug_view_ || publish_drawing)
      {
        int hist_w = 512, hist_h = 400;
        int bin_w = cvRound((double)hist_w / histogram_size_);

        drawing = cv::Mat(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

        //  Normalize the result to ( 0, drawing.rows )
        for (auto& j : hist)
        {
          cv::normalize(j, j, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());
        }

        //  Draw for each channel
        for (unsigned int j = 0; j < hist.size(); j++)
        {
          for (unsigned int i = 1; i < histogram_size_; i++)
          {
            cv::line(drawing, cv::Point(bin_w * (i - 1), hist_h - cvRound(hist[j].at<float>(i - 1))),
                     cv::Point(bin_w * i, hist_h - cvRound(hist[j].at<float>(i))),
                     cv::Scalar((j == 2 ? 255 : 0), (j == 1 ? 255 : 0), (j == 0 ? 255 : 0)), 2, 8, 0);
          }
        }
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, drawing);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      if (publish_drawing)
      {
        sensor_msgs::Image::Ptr out_img =
            cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, drawing).toImageMsg();
        img_pub_.publish(out_img);
      }
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  }

  void subscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", queue_size_, &CompareHistogramNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", queue_size_, &CompareHistogramNodelet::imageCallback, this);
  }

  void unsubscribe()  // NOLINT(modernize-use-override)
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  void onInit()  // NOLINT(modernize-use-override)
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));
    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("debug_view", debug_view_, false);
    if (pnh_->hasParam("reference_histogram"))
    {
      XmlRpc::XmlRpcValue param_val;
      pnh_->getParam("reference_histogram", param_val);
      if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        XmlRpc::XmlRpcValue param = param_val[0];
        reference_histogram_.resize(param_val.size());
        for (int j = 0; j < param_val.size(); j++)
        {
          reference_histogram_[j] = cv::Mat::zeros(param_val[0].size(), 1, CV_32F);
          for (int i = 0; i < param_val[0].size(); i++)
          {
            reference_histogram_[j].at<float>(0, i) = static_cast<double>(param_val[j][i]);
          }
          ROS_INFO_STREAM("reference_histgram[" << j << "] : " << reference_histogram_[j]);
        }
      }
    }

    window_name_ = "Histogram Window (" + ros::this_node::getName() + ")";

    reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(
        &CompareHistogramNodelet::reconfigureCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    reconfigure_server_->setCallback(f);

    img_pub_ = advertiseImage(*pnh_, "image", 1);
    msg_pub_ = advertise<std_msgs::Float64MultiArray>(*pnh_, "histogram", 1);
    distance_pub_ = advertise<std_msgs::Float64>(*pnh_, "distance", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::CompareHistogramNodelet, nodelet::Nodelet);
