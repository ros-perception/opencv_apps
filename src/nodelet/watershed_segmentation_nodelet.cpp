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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/watershed.cpp
/**
 * This program demonstrates the famous watershed segmentation algorithm in OpenCV: watershed()
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/contour.hpp>
#include <opencv_apps/msg/contour_array.hpp>
#include <opencv_apps/msg/contour_array_stamped.hpp>
#include <opencv_apps/msg/point2_d_array.hpp>

namespace opencv_apps
{
class WatershedSegmentationNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::ContourArrayStamped>::SharedPtr msg_pub_;
  rclcpp::Subscription<opencv_apps::msg::Point2DArray>::SharedPtr add_seed_points_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string window_name_, segment_name_;
  static bool need_config_update_;
  static bool on_mouse_update_;
  static int on_mouse_event_;
  static int on_mouse_x_;
  static int on_mouse_y_;
  static int on_mouse_flags_;

  cv::Mat markerMask, img_gray;
  cv::Point prevPt;

  static void onMouse(int event, int x, int y, int flags, void* /*unused*/)
  {
    on_mouse_update_ = true;
    on_mouse_event_ = event;
    on_mouse_x_ = x;
    on_mouse_y_ = y;
    on_mouse_flags_ = flags;
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
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
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::msg::ContourArrayStamped contours_msg;
      contours_msg.header = msg->header;

      // Do the work
      // std::vector<cv::Rect> faces;

      /// Initialize
      if (markerMask.empty())
      {
        cv::cvtColor(frame, markerMask, cv::COLOR_BGR2GRAY);
        markerMask = cv::Scalar::all(0);
      }
      cv::Mat tmp;
      cv::cvtColor(frame, tmp, cv::COLOR_BGR2GRAY);
      cv::cvtColor(tmp, img_gray, cv::COLOR_GRAY2BGR);

      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        cv::setMouseCallback(window_name_, onMouse, nullptr);
        if (need_config_update_)
        {
          set_parameter(rclcpp::Parameter("use_camera_info", use_camera_info_));
          need_config_update_ = false;
        }

        if (on_mouse_update_)
        {
          int event = on_mouse_event_;
          int x = on_mouse_x_;
          int y = on_mouse_y_;
          int flags = on_mouse_flags_;

          if (x < 0 || x >= frame.cols || y < 0 || y >= frame.rows)
            return;
          if (event == cv::EVENT_LBUTTONUP || !(flags & cv::EVENT_FLAG_LBUTTON))
            prevPt = cv::Point(-1, -1);
          else if (event == cv::EVENT_LBUTTONDOWN)
            prevPt = cv::Point(x, y);
          else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))
          {
            cv::Point pt(x, y);
            if (prevPt.x < 0)
              prevPt = pt;
            cv::line(markerMask, prevPt, pt, cv::Scalar::all(255), 5, 8, 0);
            cv::line(frame, prevPt, pt, cv::Scalar::all(255), 5, 8, 0);
            prevPt = pt;
            cv::imshow(window_name_, markerMask);
          }
        }
        cv::waitKey(1);
      }

      int i, j, comp_count = 0;
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::findContours(markerMask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

      if (contours.empty())
      {
        RCLCPP_WARN(get_logger(), "contours are empty");
        return;  // continue;
      }
      cv::Mat markers(markerMask.size(), CV_32S);
      markers = cv::Scalar::all(0);
      int idx = 0;
      for (; idx >= 0; idx = hierarchy[idx][0], comp_count++)
        cv::drawContours(markers, contours, idx, cv::Scalar::all(comp_count + 1), -1, 8, hierarchy, INT_MAX);

      if (comp_count == 0)
      {
        RCLCPP_WARN(get_logger(), "compCount is 0");
        return;  // continue;
      }
      for (const std::vector<cv::Point>& contour : contours)
      {
        opencv_apps::msg::Contour contour_msg;
        for (const cv::Point& j : contour)
        {
          opencv_apps::msg::Point2D point_msg;
          point_msg.x = j.x;
          point_msg.y = j.y;
          contour_msg.points.push_back(point_msg);
        }
        contours_msg.contours.push_back(contour_msg);
      }

      std::vector<cv::Vec3b> color_tab;
      for (i = 0; i < comp_count; i++)
      {
        int b = cv::theRNG().uniform(0, 255);
        int g = cv::theRNG().uniform(0, 255);
        int r = cv::theRNG().uniform(0, 255);

        color_tab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
      }

      double t = (double)cv::getTickCount();
      cv::watershed(frame, markers);
      t = (double)cv::getTickCount() - t;
      RCLCPP_INFO(get_logger(), "execution time = %gms", t * 1000. / cv::getTickFrequency());

      cv::Mat wshed(markers.size(), CV_8UC3);

      // paint the watershed image
      for (i = 0; i < markers.rows; i++)
        for (j = 0; j < markers.cols; j++)
        {
          int index = markers.at<int>(i, j);
          if (index == -1)
            wshed.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
          else if (index <= 0 || index > comp_count)
            wshed.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
          else
            wshed.at<cv::Vec3b>(i, j) = color_tab[index - 1];
        }

      wshed = wshed * 0.5 + img_gray * 0.5;

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(segment_name_, wshed);

        int c = cv::waitKey(1);
        // if( (char)c == 27 )
        //    break;
        if ((char)c == 'r')
        {
          markerMask = cv::Scalar::all(0);
        }
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(msg->header, msg->encoding, wshed).toImageMsg();
      img_pub_.publish(*out_img);
      msg_pub_->publish(contours_msg);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void addSeedPointCb(const opencv_apps::msg::Point2DArray& msg)
  {
    if (msg.points.empty())
    {
      markerMask = cv::Scalar::all(0);
    }
    else
    {
      for (const opencv_apps::msg::Point2D& point : msg.points)
      {
        cv::Point pt0(point.x, point.y);
        cv::Point pt1(pt0.x + 1, pt0.y + 1);
        cv::line(markerMask, pt0, pt1, cv::Scalar::all(255), 5, 8, 0);
      }
    }
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &WatershedSegmentationNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &WatershedSegmentationNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit WatershedSegmentationNodelet(const rclcpp::NodeOptions& options) : Nodelet("watershed_segmentation", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "roughly mark the areas to segment on the image";
    segment_name_ = "watershed transform";
    prevPt.x = -1;
    prevPt.y = -1;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&WatershedSegmentationNodelet::parametersCallback, this, std::placeholders::_1));

    add_seed_points_sub_ = create_subscription<opencv_apps::msg::Point2DArray>(
        "~/add_seed_points", 1, std::bind(&WatershedSegmentationNodelet::addSeedPointCb, this, std::placeholders::_1));
    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::ContourArrayStamped>("~/contours", 1);

    RCLCPP_INFO(get_logger(),
                "This program demonstrates the famous watershed segmentation algorithm in OpenCV: watershed()");
    RCLCPP_INFO(get_logger(), "Hot keys: ");
    RCLCPP_INFO(get_logger(), "\tESC - quit the program");
    RCLCPP_INFO(get_logger(), "\tr - restore the original image");
    RCLCPP_INFO(get_logger(), "\tw or SPACE - run watershed segmentation algorithm");
    RCLCPP_INFO(get_logger(), "\t\t(before running it, *roughly* mark the areas to segment on the image)");
    RCLCPP_INFO(get_logger(), "\t  (before that, roughly outline several markers on the image)");

    onInitPostProcess();
  }
};
bool WatershedSegmentationNodelet::need_config_update_ = false;
bool WatershedSegmentationNodelet::on_mouse_update_ = false;
int WatershedSegmentationNodelet::on_mouse_event_ = 0;
int WatershedSegmentationNodelet::on_mouse_x_ = 0;
int WatershedSegmentationNodelet::on_mouse_y_ = 0;
int WatershedSegmentationNodelet::on_mouse_flags_ = 0;
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::WatershedSegmentationNodelet)
