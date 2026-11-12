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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/camshiftdemo.cpp
/**
 * This is a demo that shows mean-shift based tracking
 * You select a color objects such as your face and it tracks it.
 * This reads from video camera (0 by default, or the camera number the user enters
 */

#include "opencv_apps/nodelet.h"

#include <iostream>
#include <mutex>
#include <string>
#include <ctype.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/rotated_rect_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

namespace opencv_apps
{
class CamShiftNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_, bproj_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::RotatedRectStamped>::SharedPtr msg_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr conf_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  // guards the state shared by doWork() and parametersCallback()
  std::mutex mutex_;
  // incremented when the user sets the rectangle, to reject the rectangle computed before it by this node
  unsigned int rect_generation_;
  // the parameters being set by setInternalParameters() on this thread, e.g. the rectangle selected with the mouse,
  // so that parametersCallback(), which set_parameters() calls on the calling thread, tells them from the user's
  struct InternalSet
  {
    const CamShiftNodelet* node;
    unsigned int generation;  // rect_generation_ when they were computed
  };
  static thread_local InternalSet internal_set_;

  std::string window_name_, histogram_name_;
  static bool need_config_update_;
  static bool on_mouse_update_;
  static int on_mouse_event_;
  static int on_mouse_x_;
  static int on_mouse_y_;

  int vmin_, vmax_, smin_;
  // rectangle to track, center x-y (same convention as opencv_apps/Rect and track_box), 0 width/height for none
  int x_, y_, width_, height_;
  bool backprojMode;
  bool selectObject;
  int showSelected;
  int trackObject;
  bool showHist;
  cv::Point origin;
  cv::Rect selection;
  bool paused;

  cv::Rect trackWindow;
  int hsize;
  float hranges[2];
  const float* phranges;
  cv::Mat hist, histimg;
  // cv::Mat hsv;

  static void onMouse(int event, int x, int y, int /*unused*/, void* /*unused*/)
  {
    on_mouse_update_ = true;
    on_mouse_event_ = event;
    on_mouse_x_ = x;
    on_mouse_y_ = y;
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    bool internal = internal_set_.node == this;
    // reject the rectangle written back by this node if the user set the rectangle after it was computed, not to
    // overwrite the user's one
    if (internal && internal_set_.generation != rect_generation_)
    {
      for (const rclcpp::Parameter& parameter : parameters)
      {
        const std::string& name = parameter.get_name();
        if (name == "x" || name == "y" || name == "width" || name == "height")
        {
          result.successful = false;
          result.reason = "the rectangle was set by the user after this one was computed";
          return result;
        }
      }
    }
    bool restart = false;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "vmin")
        vmin_ = parameter.as_int();
      else if (parameter.get_name() == "vmax")
        vmax_ = parameter.as_int();
      else if (parameter.get_name() == "smin")
        smin_ = parameter.as_int();
      else if (parameter.get_name() == "x")
        restart |= setRectValue(parameter, x_, internal);
      else if (parameter.get_name() == "y")
        restart |= setRectValue(parameter, y_, internal);
      else if (parameter.get_name() == "width")
        restart |= setRectValue(parameter, width_, internal);
      else if (parameter.get_name() == "height")
        restart |= setRectValue(parameter, height_, internal);
    }
    // (re)start the tracking when the user sets the rectangle, even to the same values (e.g. after losing the target),
    // or stop it when the user sets width/height to 0
    if (restart)
    {
      rect_generation_++;
      if (width_ > 0 && height_ > 0)
        startTrackingFromParameters();
      else
        stopTracking();
    }
    result.successful = true;
    return result;
  }

  // set value of a rectangle parameter, returns true if it is set by the user (not by setInternalParameters())
  static bool setRectValue(const rclcpp::Parameter& parameter, int& value, bool internal)
  {
    value = static_cast<int>(parameter.as_int());
    return !internal;
  }

  // start tracking the rectangle of x/y/width/height parameters, it is clipped to the image in doWork()
  void startTrackingFromParameters()
  {
    if (width_ <= 0 || height_ <= 0)
      return;
    selection = cv::Rect(x_ - width_ / 2, y_ - height_ / 2, width_, height_);
    selectObject = false;
    trackObject = -1;
  }

  void stopTracking()
  {
    selection = cv::Rect();
    selectObject = false;
    showSelected = 0;
    trackObject = 0;
    histimg = cv::Scalar::all(0);
  }

  // set parameters from this node, without restarting the tracking, computed at rect_generation_ of generation
  void setInternalParameters(const std::vector<rclcpp::Parameter>& parameters, unsigned int generation)
  {
    struct Guard
    {
      Guard(const CamShiftNodelet* node, unsigned int generation)
      {
        internal_set_ = InternalSet{ node, generation };
      }
      ~Guard()
      {
        internal_set_ = InternalSet{ nullptr, 0 };
      }
    } guard(this, generation);
    // all at once, not to mix the fields of the rectangle with an external update
    set_parameters_atomically(parameters);
  }

  const std::string& frameWithDefault(const std::string& frame, const std::string& image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
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

  static void trackbarCallback(int /*unused*/, void* /*unused*/)
  {
    need_config_update_ = true;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    // parameters to set after unlocking mutex_, as set_parameters() calls parametersCallback()
    std::vector<rclcpp::Parameter> parameters;
    unsigned int generation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      processImage(msg, parameters);
      generation = rect_generation_;
    }
    if (!parameters.empty())
      setInternalParameters(parameters, generation);
  }

  void processImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg, std::vector<rclcpp::Parameter>& parameters)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::Mat backproj;
      bool tracked = false;
      std_msgs::msg::Float64 conf_msg;

      // Messages
      opencv_apps::msg::RotatedRectStamped rect_msg;
      rect_msg.header = msg->header;

      // Do the work

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds

        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        cv::setMouseCallback(window_name_, onMouse, nullptr);
        cv::createTrackbar("Vmin", window_name_, &vmin_, 256, trackbarCallback);
        cv::createTrackbar("Vmax", window_name_, &vmax_, 256, trackbarCallback);
        cv::createTrackbar("Smin", window_name_, &smin_, 256, trackbarCallback);

        if (need_config_update_)
        {
          parameters.push_back(rclcpp::Parameter("vmin", vmin_));
          parameters.push_back(rclcpp::Parameter("vmax", vmax_));
          parameters.push_back(rclcpp::Parameter("smin", smin_));
          need_config_update_ = false;
        }
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
            {
              trackObject = -1;
              // reflect the selected rectangle to the parameters, x_ ... height_ are set by parametersCallback() when
              // they are accepted, not to be mixed with an external update coming before them
              parameters.push_back(rclcpp::Parameter("x", selection.x + selection.width / 2));
              parameters.push_back(rclcpp::Parameter("y", selection.y + selection.height / 2));
              parameters.push_back(rclcpp::Parameter("width", selection.width));
              parameters.push_back(rclcpp::Parameter("height", selection.height));
            }
            break;
        }
        on_mouse_update_ = false;
      }

      if (trackObject < 0)
      {
        // the rectangle from the parameters may be out of the image
        selection &= cv::Rect(0, 0, frame.cols, frame.rows);
        if (selection.width <= 0 || selection.height <= 0)
        {
          RCLCPP_WARN(get_logger(), "The rectangle to track (x:%d y:%d width:%d height:%d) is out of the image (%dx%d)",
                      x_, y_, width_, height_, frame.cols, frame.rows);
          trackObject = 0;
        }
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
            std::vector<float> hist_value;
            hist_value.resize(hsize);
            for (int i = 0; i < hsize; i++)
            {
              hist_value[i] = hist.at<float>(i);
            }
            parameters.push_back(rclcpp::Parameter("histogram", hist_value));

            trackWindow = selection;
            trackObject = 1;
            showSelected = 10;

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
            // trackWindow = cv::Rect(trackWindow.x - r, trackWindow.y - r,
            //                       trackWindow.x + r, trackWindow.y + r) &
            trackWindow = cv::Rect(cols / 2 - r, rows / 2 - r, cols / 2 + r, rows / 2 + r) & cv::Rect(0, 0, cols, rows);
          }

          // confidence of tracking: ratio of the back projected pixels in the track_box ellipse,
          // computed only in the bounding box of the ellipse
          {
            cv::Rect bbox = track_box.boundingRect() & cv::Rect(0, 0, backproj.cols, backproj.rows);
            int area = 0, count = 0;
            // CamShift returns an empty box when it loses the target, whose bounding box is still 1x1
            if (bbox.area() > 0 && track_box.size.width > 0 && track_box.size.height > 0)
            {
              cv::RotatedRect box_in_bbox = track_box;
              box_in_bbox.center -= cv::Point2f(bbox.tl());
              cv::Mat ellipse_mask = cv::Mat::zeros(bbox.size(), CV_8UC1), backproj_in_ellipse;
              cv::ellipse(ellipse_mask, box_in_bbox, cv::Scalar(255), -1);
              cv::bitwise_and(backproj(bbox), ellipse_mask, backproj_in_ellipse);
              area = cv::countNonZero(ellipse_mask);
              count = cv::countNonZero(backproj_in_ellipse);
            }
            conf_msg.data = area > 0 ? static_cast<double>(count) / area : 0.0;
            tracked = true;
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

      // clipped to the frame, which may be smaller than the one the selection was made in
      cv::Rect shown = selection & cv::Rect(0, 0, frame.cols, frame.rows);
      if ((showSelected > 0 || selectObject) && shown.area() > 0)
      {
        cv::Mat roi(frame, shown);
        bitwise_not(roi, roi);
      }
      if (showSelected > 0)
        showSelected--;

      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        cv::imshow(histogram_name_, histimg);

        char c = (char)cv::waitKey(1);
        // if( c == 27 )
        //  break;
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

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img1 =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      sensor_msgs::msg::Image::SharedPtr out_img2 =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, backproj).toImageMsg();
      img_pub_.publish(out_img1);
      bproj_pub_.publish(out_img2);
      if (trackObject)
        msg_pub_->publish(rect_msg);
      if (tracked)
        conf_pub_->publish(conf_msg);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &CamShiftNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &CamShiftNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit CamShiftNodelet(const rclcpp::NodeOptions& options) : Nodelet("camshift", options), rect_generation_(0)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "CamShift Demo";
    histogram_name_ = "Histogram";

    backprojMode = false;
    selectObject = false;
    showSelected = 0;
    trackObject = 0;
    showHist = true;
    paused = false;

    hsize = 16;
    hranges[0] = 0;
    hranges[1] = 180;
    phranges = hranges;
    histimg = cv::Mat::zeros(200, 320, CV_8UC3);

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    vmin_ = declare_parameter("vmin", 10, paramDescriptor("Vmin", 1, 256));
    vmax_ = declare_parameter("vmax", 256, paramDescriptor("Vmax", 1, 256));
    smin_ = declare_parameter("smin", 30, paramDescriptor("Smin", 1, 256));
    x_ = declare_parameter("x", 0, paramDescriptor("Center x of the rectangle to track", 0, 4096));
    y_ = declare_parameter("y", 0, paramDescriptor("Center y of the rectangle to track", 0, 4096));
    width_ = declare_parameter(
        "width", 0,
        paramDescriptor("Width of the rectangle to track, 0 for none (setting it to 0 stops tracking)", 0, 4096));
    height_ = declare_parameter(
        "height", 0,
        paramDescriptor("Height of the rectangle to track, 0 for none (setting it to 0 stops tracking)", 0, 4096));
    on_set_parameters_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&CamShiftNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    bproj_pub_ = advertiseImage("~/back_project", 1);
    msg_pub_ = advertise<opencv_apps::msg::RotatedRectStamped>("~/track_box", 1);
    conf_pub_ = advertise<std_msgs::msg::Float64>("~/confidence", 1);

    RCLCPP_INFO(get_logger(), "Hot keys: ");
    RCLCPP_INFO(get_logger(), "\tESC - quit the program");
    RCLCPP_INFO(get_logger(), "\tc - stop the tracking");
    RCLCPP_INFO(get_logger(), "\tb - switch to/from backprojection view");
    RCLCPP_INFO(get_logger(), "\th - show/hide object histogram");
    RCLCPP_INFO(get_logger(), "\tp - pause video");
    RCLCPP_INFO(get_logger(), "To initialize tracking, select the object with mouse, or set x/y/width/height");

    std::vector<double> hist_value = declare_parameter("histogram", std::vector<double>());
    if (hist_value.size() == hsize)
    {
      hist.create(hsize, 1, CV_32F);
      for (int i = 0; i < hsize; i++)
      {
        hist.at<float>(i) = hist_value[i];
      }
      trackObject = 1;
      trackWindow = cv::Rect(0, 0, 640, 480);  //

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
    // the rectangle given at startup has priority over the histogram
    startTrackingFromParameters();
    onInitPostProcess();
  }
};
bool CamShiftNodelet::need_config_update_ = false;
thread_local CamShiftNodelet::InternalSet CamShiftNodelet::internal_set_ = { nullptr, 0 };
bool CamShiftNodelet::on_mouse_update_ = false;
int CamShiftNodelet::on_mouse_event_ = 0;
int CamShiftNodelet::on_mouse_x_ = 0;
int CamShiftNodelet::on_mouse_y_ = 0;
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::CamShiftNodelet)
