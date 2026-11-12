/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Ryosuke Tajima.
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
*   * Neither the name of Ryosuke Tajima nor the names of its
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

// https://github.com/opencv/opencv/raw/master/samples/cpp/tutorial_code/Histograms_Matching/MatchTemplate_Demo.cpp
/**
 * @file MatchTemplate_Demo.cpp
 * @brief Sample code to use the function MatchTemplate
 * @author OpenCV team
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv_apps/nodelet.h"
#include <opencv_apps/msg/rect.hpp>
#include <opencv_apps/msg/rect_array_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cfloat>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
class MatchTemplateNodelet : public opencv_apps::Nodelet
{
  ////////////////////////////////////////////////////////
  // Dynamic Reconfigure
  ////////////////////////////////////////////////////////
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;

  std::string window_name_, template_window_name_;
  bool window_initialized_;
  // mouse events of the window, queued by onMouse() and processed in doWork() of this node, as cv::waitKey() of
  // another node in the same process may call onMouse() of this node, and may give several events at once
  std::mutex mouse_mutex_;
  std::vector<cv::Vec3i> mouse_events_;  // event, x, y
  bool selecting_;
  cv::Point origin_;
  cv::Rect selection_;

  image_transport::Publisher img_pub_, matched_img_pub_;
  image_transport::Subscriber img_sub_, template_sub_, mask_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::RectArrayStamped>::SharedPtr rects_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr score_pub_;

  std::mutex mutex_;
  bool use_camera_info_;
  int match_method_;
  double match_threshold_;
  int max_matches_;

  // the latest template and mask, from template_file/mask_file or ~template/~mask topics
  cv::Mat template_;
  cv::Mat mask_;

  static bool lowerIsBetter(int method)
  {
    return method == cv::TM_SQDIFF || method == cv::TM_SQDIFF_NORMED;
  }

  static bool isNormed(int method)
  {
    return method == cv::TM_SQDIFF_NORMED || method == cv::TM_CCORR_NORMED || method == cv::TM_CCOEFF_NORMED;
  }

  // similarity in [0, 1] (or [-1, 1] for TM_CCOEFF_NORMED) of normalized methods, compared with match_threshold
  static double similarity(int method, double score)
  {
    return method == cv::TM_SQDIFF_NORMED ? 1.0 - score : score;
  }

  static bool methodAcceptsMask(int method)
  {
#if CV_MAJOR_VERSION > 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION >= 2)
    return true;  // all the methods support the mask
#else
    return method == cv::TM_SQDIFF || method == cv::TM_CCORR_NORMED;
#endif
  }

  static void onMouse(int event, int x, int y, int /*unused*/, void* userdata)
  {
    MatchTemplateNodelet* self = static_cast<MatchTemplateNodelet*>(userdata);
    std::lock_guard<std::mutex> lock(self->mouse_mutex_);
    self->mouse_events_.push_back(cv::Vec3i(event, x, y));
  }

  // select the template by the queued mouse events, on frame
  void processMouseEvents(const cv::Mat& frame)
  {
    std::vector<cv::Vec3i> events;
    {
      std::lock_guard<std::mutex> lock(mouse_mutex_);
      events.swap(mouse_events_);
    }
    for (const cv::Vec3i& e : events)
    {
      int event = e[0], x = e[1], y = e[2];
      if (selecting_)
      {
        selection_.x = std::min(x, origin_.x);
        selection_.y = std::min(y, origin_.y);
        selection_.width = std::abs(x - origin_.x);
        selection_.height = std::abs(y - origin_.y);
        selection_ &= cv::Rect(0, 0, frame.cols, frame.rows);
      }
      if (event == cv::EVENT_LBUTTONDOWN)
      {
        origin_ = cv::Point(x, y);
        selection_ = cv::Rect(x, y, 0, 0);
        selecting_ = true;
      }
      else if (event == cv::EVENT_LBUTTONUP)
      {
        selecting_ = false;
        if (selection_.width > 0 && selection_.height > 0)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          template_ = frame(selection_).clone();
        }
      }
    }
  }

  // binary (0 or 1) mask with the channels of the template, as OpenCV 3.x requires
  static cv::Mat toMask(const cv::Mat& gray, int channels)
  {
    cv::Mat mask;
    cv::threshold(gray, mask, 0, 1, cv::THRESH_BINARY);
    if (channels == 3)
      cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
    return mask;
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

  void templateCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try
    {
      cv::Mat templ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      std::lock_guard<std::mutex> lock(mutex_);
      template_ = templ;
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to convert the template image: %s", e.what());
    }
  }

  void maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try
    {
      cv::Mat mask = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
      std::lock_guard<std::mutex> lock(mutex_);
      mask_ = toMask(mask, 1);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to convert the mask image: %s", e.what());
    }
  }

  // read the template image of template_file, false with error if it cannot be read
  static bool readTemplate(const std::string& file, cv::Mat& templ, std::string& error)
  {
    templ = cv::imread(file, cv::IMREAD_COLOR);
    if (templ.empty())
    {
      error = "Failed to read the template image " + file;
      return false;
    }
    return true;
  }

  // read the mask image of mask_file, false with error if it cannot be read
  static bool readMask(const std::string& file, cv::Mat& mask, std::string& error)
  {
    cv::Mat image = cv::imread(file, cv::IMREAD_GRAYSCALE);
    if (image.empty())
    {
      error = "Failed to read the mask image " + file;
      return false;
    }
    mask = toMask(image, 1);
    return true;
  }

  // the template and the mask are raw images, not of the image_transport of the input images
  template <class F>
  image_transport::Subscriber subscribeRawImage(const std::string& topic, F fp)
  {
    auto callback = std::bind(fp, this, std::placeholders::_1);
#if __has_include(<image_transport/node_interfaces.hpp>)
    return image_transport::create_subscription(*this, topic, callback, "raw", subscriberQos(1));
#else
    return image_transport::create_subscription(this, topic, callback, "raw", subscriberQos(1).get_rmw_qos_profile());
#endif
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &MatchTemplateNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &MatchTemplateNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    // read the template and mask files first, not to change anything when one of them cannot be read
    cv::Mat templ, mask;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "template_file" && !parameter.as_string().empty() &&
          !readTemplate(parameter.as_string(), templ, result.reason))
        return result;
      if (parameter.get_name() == "mask_file" && !parameter.as_string().empty() &&
          !readMask(parameter.as_string(), mask, result.reason))
        return result;
    }
    if (!templ.empty())
      template_ = templ;
    if (!mask.empty())
      mask_ = mask;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "match_method")
        match_method_ = parameter.as_int();
      else if (parameter.get_name() == "match_threshold")
        match_threshold_ = parameter.as_double();
      else if (parameter.get_name() == "max_matches")
        max_matches_ = parameter.as_int();
    }
    result.successful = true;
    return result;
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    try
    {
      cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Select the template with mouse in the debug window
      if (debug_view_ && !window_initialized_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(window_name_, onMouse, this);
        window_initialized_ = true;
      }
      processMouseEvents(frame);

      cv::Mat templ, mask;
      int match_method, max_matches;
      double match_threshold;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        templ = template_;
        mask = mask_;
        match_method = match_method_;
        max_matches = max_matches_;
        match_threshold = match_threshold_;
      }

      if (templ.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "No template image, set ~template_file, publish ~template, or select it with mouse");
        showDebugView(frame, templ);
        return;
      }
      if (templ.cols > frame.cols || templ.rows > frame.rows)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Template (%dx%d) is larger than the image (%dx%d)",
                             templ.cols, templ.rows, frame.cols, frame.rows);
        showDebugView(frame, templ);
        return;
      }
      if (!mask.empty())
      {
        if (mask.size() != templ.size())
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Mask (%dx%d) is not the size of the template (%dx%d), the mask is not used", mask.cols,
                               mask.rows, templ.cols, templ.rows);
          mask = cv::Mat();
        }
        else if (!methodAcceptsMask(match_method))
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Matching method %d does not support the mask with OpenCV %s, the mask is not used",
                               match_method, CV_VERSION);
          mask = cv::Mat();
        }
        else if (mask.channels() != templ.channels())
        {
          cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
        }
      }

      /// Do the Matching
      cv::Mat score;
      if (!mask.empty())
        cv::matchTemplate(frame, templ, score, match_method, mask);
      else
        cv::matchTemplate(frame, templ, score, match_method);

      /// Masked and normalized methods can give inf or nan on flat regions
      // (comparisons with nan are false, so this is 0 for nan and inf)
      cv::Mat finite = cv::abs(score) <= FLT_MAX;

      /// Find the best match, then the next best matches out of the previous matches (non-maximum suppression)
      bool lower_is_better = lowerIsBetter(match_method);
      cv::Mat valid = finite.clone();
      opencv_apps::msg::RectArrayStamped rects_msg;
      rects_msg.header = msg->header;
      rects_msg.header.frame_id = input_frame_from_msg;
      std::vector<cv::Point> matches;
      double best_score = std::numeric_limits<double>::quiet_NaN();
      for (int i = 0; i < max_matches; i++)
      {
        double min_val, max_val;
        cv::Point min_loc(-1, -1), max_loc(-1, -1);
        cv::minMaxLoc(score, &min_val, &max_val, &min_loc, &max_loc, valid);
        cv::Point loc = lower_is_better ? min_loc : max_loc;
        double val = lower_is_better ? min_val : max_val;
        if (loc.x < 0)  // no valid score left
          break;
        if (i == 0)
          best_score = val;
        // max_matches 1 gives the best match, more than 1 the matches above match_threshold
        if (max_matches > 1 && isNormed(match_method) && similarity(match_method, val) < match_threshold)
          break;
        matches.push_back(loc);
        opencv_apps::msg::Rect rect;
        rect.x = loc.x + templ.cols / 2.0;
        rect.y = loc.y + templ.rows / 2.0;
        rect.width = templ.cols;
        rect.height = templ.rows;
        rects_msg.rects.push_back(rect);
        // suppress the matches whose rectangles overlap more than half of the template with this one, the overlap
        // of the rectangle at loc + (dx, dy) is (width - |dx|) * (height - |dy|)
        cv::Rect around =
            cv::Rect(loc.x - templ.cols + 1, loc.y - templ.rows + 1, 2 * templ.cols - 1, 2 * templ.rows - 1) &
            cv::Rect(0, 0, valid.cols, valid.rows);
        for (int y = around.y; y < around.y + around.height; y++)
        {
          uchar* row = valid.ptr<uchar>(y);
          for (int x = around.x; x < around.x + around.width; x++)
          {
            double overlap = static_cast<double>(templ.cols - std::abs(x - loc.x)) * (templ.rows - std::abs(y - loc.y));
            if (overlap * 2 > static_cast<double>(templ.cols) * templ.rows)
              row[x] = 0;
          }
        }
      }

      /// Show what you got
      for (size_t i = 0; i < matches.size(); i++)
      {
        cv::rectangle(frame, matches[i], matches[i] + cv::Point(templ.cols, templ.rows), cv::Scalar(0, 0, 255),
                      i == 0 ? 3 : 2);
      }
      cv::putText(frame, cv::format("score %.3f", best_score), cv::Point(5, frame.rows - 10), cv::FONT_HERSHEY_SIMPLEX,
                  0.6, matches.empty() ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255), 2);

      /// Score map as an 8 bit image, white is the best match
      cv::Mat score_img;
      double min_val, max_val;
      cv::minMaxLoc(score, &min_val, &max_val, nullptr, nullptr, finite);
      score.setTo(cv::Scalar(lower_is_better ? max_val : min_val), ~finite);
      cv::normalize(score, score_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      if (lower_is_better)
        score_img = 255 - score_img;

      showDebugView(frame, templ);

      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg());
      matched_img_pub_.publish(
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, score_img).toImageMsg());
      rects_pub_->publish(rects_msg);
      std_msgs::msg::Float64 score_msg;
      score_msg.data = best_score;
      score_pub_->publish(score_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }
  }

  // show the frame (with the selection being dragged, drawn on a copy) and the template
  void showDebugView(const cv::Mat& frame, const cv::Mat& templ)
  {
    if (!debug_view_)
      return;
    cv::Mat view = frame;
    cv::Rect selection = selection_ & cv::Rect(0, 0, frame.cols, frame.rows);
    if (selecting_ && selection.width > 0 && selection.height > 0)
    {
      view = frame.clone();
      cv::Mat roi(view, selection);
      cv::bitwise_not(roi, roi);
    }
    cv::imshow(window_name_, view);
    if (!templ.empty())
    {
      cv::namedWindow(template_window_name_, cv::WINDOW_AUTOSIZE);
      cv::imshow(template_window_name_, templ);
    }
    cv::waitKey(1);
  }

public:
  ~MatchTemplateNodelet()
  {
    // the windows may outlive this node, e.g. unloaded from a component container, then their mouse callback must
    // not call this node
    if (window_initialized_)
    {
      try
      {
        cv::setMouseCallback(window_name_, nullptr, nullptr);
        cv::destroyWindow(window_name_);
        cv::destroyWindow(template_window_name_);
      }
      catch (const cv::Exception&)
      {
      }
    }
  }

  explicit MatchTemplateNodelet(const rclcpp::NodeOptions& options) : Nodelet("match_template", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    // with the namespace, not to share the windows with another match_template node in the same process
    window_name_ = std::string(get_fully_qualified_name()) + " (Match Template Demo)";
    template_window_name_ = std::string(get_fully_qualified_name()) + " (Template)";
    window_initialized_ = false;
    selecting_ = false;

    // the files are read again when the parameters are set, and an unreadable file is rejected
    std::string template_file =
        declare_parameter("template_file", "", paramDescriptor("File name of the template image"));
    std::string mask_file = declare_parameter(
        "mask_file", "", paramDescriptor("File name of the mask image of the template, non-zero pixels are used"));
    std::string error;
    if (!template_file.empty() && !readTemplate(template_file, template_, error))
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    if (!mask_file.empty() && !readMask(mask_file, mask_, error))
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());

    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    match_method_ = declare_parameter(
        "match_method", 5,
        paramDescriptor("Matching Method, 0: TM_SQDIFF, 1: TM_SQDIFF_NORMED, 2: TM_CCORR, 3: TM_CCORR_NORMED, 4: "
                        "TM_CCOEFF, 5: TM_CCOEFF_NORMED",
                        0, 5));
    match_threshold_ = declare_parameter(
        "match_threshold", 0.0,
        paramDescriptor("Minimum similarity of a match, the score of TM_CCORR_NORMED and TM_CCOEFF_NORMED, 1 - score "
                        "of TM_SQDIFF_NORMED; not used by the methods which are not normalized",
                        0.0, 1.0));
    max_matches_ = declare_parameter(
        "max_matches", 1,
        paramDescriptor("Maximum number of the matches. 1: the best match only, more than 1: the matches above "
                        "match_threshold, after a non-maximum suppression",
                        1, 100));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&MatchTemplateNodelet::parametersCallback, this, std::placeholders::_1));

    // the latest template and mask are kept, even if the output topics are not subscribed
    template_sub_ = subscribeRawImage("~/template", &MatchTemplateNodelet::templateCallback);
    mask_sub_ = subscribeRawImage("~/mask", &MatchTemplateNodelet::maskCallback);

    img_pub_ = advertiseImage("~/image", 1);
    matched_img_pub_ = advertiseImage("~/matched_image", 1);
    rects_pub_ = advertise<opencv_apps::msg::RectArrayStamped>("~/output", 1);
    score_pub_ = advertise<std_msgs::msg::Float64>("~/score", 1);
    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::MatchTemplateNodelet)
