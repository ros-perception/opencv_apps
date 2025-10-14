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

#include <rclcpp/rclcpp.hpp>
#include "opencv_apps/nodelet.h"
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <iostream>
#include <ctype.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_apps/msg/rotated_rect_stamped.hpp"

namespace opencv_apps
{
class CamShiftNodelet : public opencv_apps::Nodelet
{
  std::shared_ptr<image_transport::Publisher> img_pub_, bproj_pub_;
  std::shared_ptr<image_transport::Subscriber> img_sub_;
  std::shared_ptr<image_transport::CameraSubscriber> cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::RotatedRectStamped>::SharedPtr msg_pub_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_, histogram_name_;
  static bool on_mouse_update_;
  static int on_mouse_event_;
  static int on_mouse_x_;
  static int on_mouse_y_;

  int vmin_, vmax_, smin_;
  bool backprojMode;
  bool selectObject;
  int trackObject;
  bool showHist;
  cv::Point origin;
  cv::Rect selection;
  bool paused;
  bool use_camera_info_;

  cv::Rect trackWindow;
  int hsize;
  float hranges[2];
  const float* phranges;
  cv::Mat hist, histimg;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  static void onMouse(int event, int x, int y, int /*unused*/, void* /*unused*/)
  {
    on_mouse_update_ = true;
    on_mouse_event_ = event;
    on_mouse_x_ = x;
    on_mouse_y_ = y;
  }

  rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters)
    {
      if (param.get_name() == "vmin")
      {
        vmin_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated vmin to %d", vmin_);
      }
      else if (param.get_name() == "vmax")
      {
        vmax_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated vmax to %d", vmax_);
      }
      else if (param.get_name() == "smin")
      {
        smin_ = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated smin to %d", smin_);
      }
    }
    return result;
  }

  void imageCallbackWithInfo(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
  {
    doWork(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    doWork(msg, msg->header.frame_id);
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    try
    {
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::Mat backproj;

      opencv_apps::msg::RotatedRectStamped rect_msg;
      rect_msg.header = msg->header;

      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(window_name_, onMouse, nullptr);
        cv::createTrackbar("Vmin", window_name_, &vmin_, 256, nullptr);
        cv::createTrackbar("Vmax", window_name_, &vmax_, 256, nullptr);
        cv::createTrackbar("Smin", window_name_, &smin_, 256, nullptr);
      }

      if (on_mouse_update_)
      {
        int event = on_mouse_event_;
        int x = on_mouse_x_;
        int y = on_mouse_y_;

        if (selectObject)
        {
          selection.x = MIN(x, origin.x);
          selection.y = MIN(y, origin.y);
          selection.width = std::abs(x - origin.x);
          selection.height = std::abs(y - origin.y);
          selection &= cv::Rect(0, 0, frame.cols, frame.rows);
        }

        switch (event)
        {
          case cv::EVENT_LBUTTONDOWN:
            origin = cv::Point(x, y);
            selection = cv::Rect(x, y, 0, 0);
            selectObject = true;
            break;
          case cv::EVENT_LBUTTONUP:
            selectObject = false;
            if (selection.width > 0 && selection.height > 0)
              trackObject = -1;
            break;
        }
        on_mouse_update_ = false;
      }

      if (!paused)
      {
        cv::Mat hsv, hue, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        if (trackObject)
        {
          int vmin = vmin_, vmax = vmax_;

          cv::inRange(hsv, cv::Scalar(0, smin_, MIN(vmin, vmax)), cv::Scalar(180, 256, MAX(vmin, vmax)), mask);
          int ch[] = { 0, 0 };
          hue.create(hsv.size(), hsv.depth());
          cv::mixChannels(&hsv, 1, &hue, 1, ch, 1);

          if (trackObject < 0)
          {
            cv::Mat roi(hue, selection), maskroi(mask, selection);
            cv::calcHist(&roi, 1, nullptr, maskroi, hist, 1, &hsize, &phranges);
            cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);

            std::vector<double> hist_value;
            hist_value.resize(hsize);
            for (int i = 0; i < hsize; i++)
            {
              hist_value[i] = hist.at<float>(i);
            }
            this->set_parameter(rclcpp::Parameter("histogram", hist_value));

            trackWindow = selection;
            trackObject = 1;

            histimg = cv::Scalar::all(0);
            int bin_w = histimg.cols / hsize;
            cv::Mat buf(1, hsize, CV_8UC3);
            for (int i = 0; i < hsize; i++)
              buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i * 180. / hsize), 255, 255);
            cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);

            for (int i = 0; i < hsize; i++)
            {
              int val = cv::saturate_cast<int>(hist.at<float>(i) * histimg.rows / 255);
              cv::rectangle(histimg, cv::Point(i * bin_w, histimg.rows), cv::Point((i + 1) * bin_w, histimg.rows - val),
                            cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8);
            }
          }

          cv::calcBackProject(&hue, 1, nullptr, hist, backproj, &phranges);
          backproj &= mask;
          cv::RotatedRect track_box = cv::CamShift(
              backproj, trackWindow, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1));

          if (trackWindow.area() <= 1)
          {
            int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
            trackWindow = cv::Rect(cols / 2 - r, rows / 2 - r, cols / 2 + r, rows / 2 + r) & cv::Rect(0, 0, cols, rows);
          }

          if (backprojMode)
            cv::cvtColor(backproj, frame, cv::COLOR_GRAY2BGR);
#ifndef CV_VERSION_EPOCH
          cv::ellipse(frame, track_box, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
#else
          cv::ellipse(frame, track_box, cv::Scalar(0, 0, 255), 3, CV_AA);
#endif

          rect_msg.rect.angle = track_box.angle;
          opencv_apps::msg::Point2D point_msg;
          opencv_apps::msg::Size size_msg;
          point_msg.x = track_box.center.x;
          point_msg.y = track_box.center.y;
          size_msg.width = track_box.size.width;
          size_msg.height = track_box.size.height;
          rect_msg.rect.center = point_msg;
          rect_msg.rect.size = size_msg;
        }
      }
      else if (trackObject < 0)
        paused = false;

      if (selectObject && selection.width > 0 && selection.height > 0)
      {
        cv::Mat roi(frame, selection);
        bitwise_not(roi, roi);
      }

      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        if (showHist)
          cv::imshow(histogram_name_, histimg);

        char c = (char)cv::waitKey(1);
        switch (c)
        {
          case 'b':
            backprojMode = !backprojMode;
            break;
          case 'c':
            trackObject = 0;
            histimg = cv::Scalar::all(0);
            break;
          case 'h':
            showHist = !showHist;
            if (!showHist)
              cv::destroyWindow(histogram_name_);
            else
              cv::namedWindow(histogram_name_, 1);
            break;
          case 'p':
            paused = !paused;
            break;
          default:;
        }
      }

      sensor_msgs::msg::Image::SharedPtr out_img1 = cv_bridge::CvImage(msg->header, msg->encoding, frame).toImageMsg();
      sensor_msgs::msg::Image::SharedPtr out_img2 =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, backproj).toImageMsg();
      img_pub_->publish(*out_img1);
      bproj_pub_->publish(*out_img2);
      if (trackObject)
        msg_pub_->publish(rect_msg);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(),
                   e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()
  {
    RCLCPP_DEBUG(this->get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
    {
      cam_sub_ = std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this, "image",
          std::bind(&CamShiftNodelet::imageCallbackWithInfo, this,
                    std::placeholders::_1, std::placeholders::_2),
          "raw"));
    }
    else
    {
      img_sub_ = std::make_shared<image_transport::Subscriber>(
        image_transport::create_subscription(
          this, "image",
          std::bind(&CamShiftNodelet::imageCallback, this, std::placeholders::_1),
          "raw"));
    }
  }

  void unsubscribe()
  {
    RCLCPP_DEBUG(this->get_logger(), "Unsubscribing from image topic.");
    img_sub_.reset();
    cam_sub_.reset();
  }

public:
  CamShiftNodelet(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Nodelet("camshift", options)
  {
  }

  void onInit() override
  {
    Nodelet::onInit();

    this->declare_parameter("queue_size", 3);
    this->declare_parameter("debug_view", false);
    this->declare_parameter("use_camera_info", false);
    this->declare_parameter("vmin", 10);
    this->declare_parameter("vmax", 256);
    this->declare_parameter("smin", 30);
    this->declare_parameter("histogram", std::vector<double>());

    this->get_parameter("queue_size", queue_size_);
    this->get_parameter("debug_view", debug_view_);
    this->get_parameter("use_camera_info", use_camera_info_);
    this->get_parameter("vmin", vmin_);
    this->get_parameter("vmax", vmax_);
    this->get_parameter("smin", smin_);

    if (debug_view_)
    {
      RCLCPP_INFO(this->get_logger(), "debug_view: %s", debug_view_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "debug_view is enabled, setting always_subscribe to true");
      always_subscribe_ = true;
      RCLCPP_INFO(this->get_logger(), "always_subscribe: %s", always_subscribe_ ? "true" : "false");
    }
    prev_stamp_ = rclcpp::Time(0);

    window_name_ = "CamShift Demo";
    histogram_name_ = "Histogram";

    backprojMode = false;
    selectObject = false;
    trackObject = 0;
    showHist = true;
    paused = false;

    hsize = 16;
    hranges[0] = 0;
    hranges[1] = 180;
    phranges = hranges;
    histimg = cv::Mat::zeros(200, 320, CV_8UC3);

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CamShiftNodelet::parameterCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("image_out", 1);
    bproj_pub_ = advertiseImage("back_project", 1);
    msg_pub_ = this->create_publisher<opencv_apps::msg::RotatedRectStamped>("track_box", 1);

    RCLCPP_INFO(this->get_logger(), "Hot keys:");
    RCLCPP_INFO(this->get_logger(), "      ESC - quit the program");
    RCLCPP_INFO(this->get_logger(), "      c - stop the tracking");
    RCLCPP_INFO(this->get_logger(), "      b - switch to/from backprojection view");
    RCLCPP_INFO(this->get_logger(), "      h - show/hide object histogram");
    RCLCPP_INFO(this->get_logger(), "      p - pause video");
    RCLCPP_INFO(this->get_logger(), "To initialize tracking, select the object with mouse");

    std::vector<double> hist_value;
    this->get_parameter("histogram", hist_value);
    if (hist_value.size() == static_cast<size_t>(hsize))
    {
      hist.create(hsize, 1, CV_32F);
      for (int i = 0; i < hsize; i++)
      {
        hist.at<float>(i) = hist_value[i];
      }
      trackObject = 1;
      trackWindow = cv::Rect(0, 0, 640, 480);

      histimg = cv::Scalar::all(0);
      int bin_w = histimg.cols / hsize;
      cv::Mat buf(1, hsize, CV_8UC3);
      for (int i = 0; i < hsize; i++)
        buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i * 180. / hsize), 255, 255);
      cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);

      for (int i = 0; i < hsize; i++)
      {
        int val = cv::saturate_cast<int>(hist.at<float>(i) * histimg.rows / 255);
        cv::rectangle(histimg, cv::Point(i * bin_w, histimg.rows), cv::Point((i + 1) * bin_w, histimg.rows - val),
                      cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8);
      }
    }

    onInitPostProcess();
  }
};

bool CamShiftNodelet::on_mouse_update_ = false;
int CamShiftNodelet::on_mouse_event_ = 0;
int CamShiftNodelet::on_mouse_x_ = 0;
int CamShiftNodelet::on_mouse_y_ = 0;

}  // namespace opencv_apps

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::CamShiftNodelet)

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<opencv_apps::CamShiftNodelet>();
  node->onInit();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
