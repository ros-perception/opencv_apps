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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/peopledetect.cpp
/**
 * Demonstrate the use of the HoG descriptor using
 * HOGDescriptor::hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <opencv_apps/msg/rect.hpp>
#include <opencv_apps/msg/rect_array_stamped.hpp>

namespace opencv_apps
{
class PeopleDetectNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::RectArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  cv::HOGDescriptor hog_;

  bool use_camera_info_;
  double hit_threshold_;
  int win_stride_;
  int padding_;
  double scale0_;
  int group_threshold_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "hit_threshold")
        hit_threshold_ = parameter.as_double();
      else if (parameter.get_name() == "win_stride")
        win_stride_ = parameter.as_int();
      else if (parameter.get_name() == "padding")
        padding_ = parameter.as_int();
      else if (parameter.get_name() == "scale0")
        scale0_ = parameter.as_double();
      else if (parameter.get_name() == "group_threshold")
        group_threshold_ = parameter.as_int();
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
      opencv_apps::msg::RectArrayStamped found_msg;
      found_msg.header = msg->header;

      // Do the work
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      }

      std::vector<cv::Rect> found, found_filtered;
      double t = (double)cv::getTickCount();
      // run the detector with default parameters. to get a higher hit-rate
      // (and more false alarms, respectively), decrease the hitThreshold and
      // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
      hog_.detectMultiScale(frame, found, hit_threshold_, cv::Size(win_stride_, win_stride_),
                            cv::Size(padding_, padding_), scale0_, group_threshold_);
      t = (double)cv::getTickCount() - t;
      RCLCPP_INFO(get_logger(), "tdetection time = %gms", t * 1000. / cv::getTickFrequency());
      size_t i, j;
      for (i = 0; i < found.size(); i++)
      {
        cv::Rect r = found[i];
        for (j = 0; j < found.size(); j++)
          if (j != i && (r & found[j]) == r)
            break;
        if (j == found.size())
          found_filtered.push_back(r);
      }
      for (i = 0; i < found_filtered.size(); i++)
      {
        cv::Rect r = found_filtered[i];
        // the HOG detector returns slightly larger rectangles than the real objects.
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width * 0.1);
        r.width = cvRound(r.width * 0.8);
        r.y += cvRound(r.height * 0.07);
        r.height = cvRound(r.height * 0.8);
        cv::rectangle(frame, r.tl(), r.br(), cv::Scalar(0, 255, 0), 3);

        opencv_apps::msg::Rect rect_msg;
        rect_msg.x = r.x;
        rect_msg.y = r.y;
        rect_msg.width = r.width;
        rect_msg.height = r.height;
        found_msg.rects.push_back(rect_msg);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow(window_name_, frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(msg->header, msg->encoding, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(found_msg);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &PeopleDetectNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &PeopleDetectNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit PeopleDetectNodelet(const rclcpp::NodeOptions& options) : Nodelet("people_detect", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "people detector";

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    hit_threshold_ = declare_parameter(
        "hit_threshold", 0.0,
        paramDescriptor("Threshold for the distance between features and SVM classifying plane.", 0.0, 10.0));
    win_stride_ = declare_parameter("win_stride", 8,
                                    paramDescriptor("Window stride. It must be a multiple of block stride.", 1, 100));
    padding_ = declare_parameter(
        "padding", 32,
        paramDescriptor("Mock parameter to keep the CPU interface compatibility. It must be (0,0).", 0, 128));
    scale0_ =
        declare_parameter("scale0", 1.05, paramDescriptor("Coefficient of the detection window increase.", 1.0, 100.0));
    group_threshold_ = declare_parameter(
        "group_threshold", 2,
        paramDescriptor("Coefficient to regulate the similarity threshold. When detected, some objects can be covered "
                        "by many rectangles. 0 means not to perform grouping.",
                        0, 100));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&PeopleDetectNodelet::parametersCallback, this, std::placeholders::_1));

    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::RectArrayStamped>("~/found", 1);

    onInitPostProcess();
  }
};
bool PeopleDetectNodelet::need_config_update_ = false;
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::PeopleDetectNodelet)
