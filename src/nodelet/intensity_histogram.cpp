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

#include <vector>

#include <cv_bridge/cv_bridge.h>
//#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <opencv_apps/nodelet.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace opencv_apps
{

namespace intensity_histogram
{

static const std::string OPENCV_WINDOW = "Image histogram";

cv::Mat grayHistogram(const cv_bridge::CvImageConstPtr& img)
{
  /* Inspired by
   * https://github.com/opencv/opencv/tree/3.4/samples/cpp/tutorial_code/Histograms_Matching/calcHist_Demo.cpp. */
  constexpr bool uniform = true;
  constexpr bool accumulate = false;
  constexpr int bin_count = 256;

  float range[] = { 0, 256 }; //the upper boundary is exclusive
  const float* hist_range = { range };
  cv::Mat intensity_hist;
  cv::calcHist(&img->image, 1, nullptr, cv::Mat(), intensity_hist, 1, &bin_count, &hist_range, uniform, accumulate);
  int hist_w = 512;
  int hist_h = 400;
  int bin_w = cvRound(static_cast<double>(hist_w / bin_count));
  cv::Mat hist_image(hist_h, hist_w, CV_8UC1, cv::Scalar(0));
  cv::normalize(intensity_hist, intensity_hist, 0, hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
  for (unsigned int i = 1; i < bin_count; i++)
  {
    cv::line(hist_image,
        cv::Point(bin_w * (i - 1), hist_h - cvRound(intensity_hist.at<float>(i - 1))),
        cv::Point(bin_w * i, hist_h - cvRound(intensity_hist.at<float>(i))),
        cv::Scalar(255), 2, 8, 0);
  }
  return hist_image;
}

cv::Mat bgrHistogram(const cv_bridge::CvImageConstPtr& img)
{
  /* Inspired by
   * https://github.com/opencv/opencv/tree/3.4/samples/cpp/tutorial_code/Histograms_Matching/calcHist_Demo.cpp. */
  constexpr bool uniform = true;
  constexpr bool accumulate = false;
  constexpr int bin_count = 256;

  std::vector<cv::Mat> bgr_planes;
  cv::split(img->image, bgr_planes);

  float range[] = {0, 256}; //the upper boundary is exclusive
  const float* hist_range = {range};
  cv::Mat b_hist;
  cv::Mat g_hist;
  cv::Mat r_hist;
  cv::calcHist(&bgr_planes[0], 1, nullptr, cv::Mat(), b_hist, 1, &bin_count, &hist_range, uniform, accumulate);
  cv::calcHist(&bgr_planes[1], 1, nullptr, cv::Mat(), g_hist, 1, &bin_count, &hist_range, uniform, accumulate);
  cv::calcHist(&bgr_planes[2], 1, nullptr, cv::Mat(), r_hist, 1, &bin_count, &hist_range, uniform, accumulate);
  int hist_w = 512;
  int hist_h = 400;
  int bin_w = cvRound(static_cast<double>(hist_w / bin_count));
  cv::Mat hist_image(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::normalize(b_hist, b_hist, 0, hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
  cv::normalize(g_hist, g_hist, 0, hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
  cv::normalize(r_hist, r_hist, 0, hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
  for (unsigned int i = 1; i < bin_count; i++)
  {
    cv::line(hist_image,
        cv::Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
        cv::Point(bin_w * i, hist_h - cvRound(b_hist.at<float>(i))),
        cv::Scalar(255, 0, 0), 2, 8, 0);
    cv::line(hist_image,
        cv::Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
        cv::Point(bin_w * i, hist_h - cvRound(g_hist.at<float>(i))),
        cv::Scalar(0, 255, 0), 2, 8, 0);
    cv::line(hist_image,
        cv::Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
        cv::Point(bin_w * i, hist_h - cvRound(r_hist.at<float>(i))),
        cv::Scalar(0, 0, 255), 2, 8, 0);
  }
  return hist_image;
}

class IntensityHistogram
{

public:

  IntensityHistogram() :
    it_(nh_)
  {
    ros::NodeHandle private_nh("~");
    private_nh.param("queue_size", queue_size_, 1);
    private_nh.param("debug_view", debug_view_, false);

    image_sub_ = it_.subscribe("image", queue_size_, &IntensityHistogram::imageCallback, this);
    hist_pub_ = it_.advertise("image_hist", queue_size_);

    if (debug_view_)
    {
      cv::namedWindow(OPENCV_WINDOW);
    }
  }

  ~IntensityHistogram()
  {
    if (debug_view_)
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat out_img;
    sensor_msgs::Image::Ptr out_img_msg;
    if (cv_ptr->image.channels() == 1)
    {
      out_img = grayHistogram(cv_ptr);
      out_img_msg = cv_bridge::CvImage(
        msg->header, sensor_msgs::image_encodings::MONO8, out_img).toImageMsg();

    }
    else
    {
      out_img = bgrHistogram(cv_ptr);
      out_img_msg = cv_bridge::CvImage(
        msg->header, sensor_msgs::image_encodings::RGB8, out_img).toImageMsg();
    }

    if (debug_view_)
    {
      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, out_img);
      cv::waitKey(3);
    }

    hist_pub_.publish(out_img_msg);
  }

private:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher hist_pub_;
  int queue_size_;
  bool debug_view_;
};

} /* namespace intensity_histogram. */

class IntensityHistogramNodelet : public nodelet::Nodelet
{

public:

  virtual void onInit()  // NOLINT(modernize-use-override)
  {
    intensity_histogram::IntensityHistogram ih;
    ros::spin();
  }
};

} /* namespace opencv_apps. */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(opencv_apps::IntensityHistogramNodelet, nodelet::Nodelet);
