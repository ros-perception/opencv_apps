// -*- mode: c++ -*-
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

/**
 * Compute the histogram of each channel of the image (BGR, or gray for single channel images), and compare it with
 * a reference histogram.
 *
 * - ~/histogram (std_msgs/Float64MultiArray): the histogram of each channel, normalized to [0, 1] (NORM_MINMAX),
 *   laid out as [channel][bin].
 * - ~/distance (std_msgs/Float64): the result of cv::compareHist() between ~reference_histogram and the histogram
 *   of all the channels, i.e. the histograms of the channels concatenated into one histogram of
 *   channels * histogram_size bins. For Chi-Square and Intersection, this is the sum of the results of the channels.
 * - ~/channel_distances (std_msgs/Float64MultiArray): the result of cv::compareHist() for each channel.
 * - ~/image (sensor_msgs/Image): the plot of the histograms.
 *
 * ~reference_histogram is a flat list in the same layout as ~/histogram, so it must have channels * histogram_size
 * values; the distances are published only then. On ROS2, it may be a double[] or an integer[] parameter, or a string
 * of numbers separated by commas and/or spaces, optionally in brackets (e.g. "[0.1, 0.5, 1, ...]", as
 * launch/compare_histogram.launch.xml passes it); an empty string or array disables the comparison.
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <cmath>
#include <locale>
#include <sstream>

#include "opencv_apps/nodelet.h"

namespace opencv_apps
{
enum CompareHistogramType
{
  CompareHistogram_Correlation = 0,
  CompareHistogram_Chi_Square = 1,
  CompareHistogram_Intersection = 2,
  CompareHistogram_Bhattacharyya = 3
};

class CompareHistogramNodelet : public opencv_apps::Nodelet
{
  std::string window_name_;
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr histogram_pub_, channel_distances_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  std::mutex mutex_;
  int queue_size_;
  bool debug_view_;
  bool use_camera_info_;
  int histogram_comparison_type_;
  int histogram_size_;
  bool accumulate_;
  std::vector<double> reference_histogram_;

  // (accumulated) histogram of each channel
  std::vector<cv::Mat> hist_;

  /** @brief
   * Values of reference_histogram, which is a double[], an integer[] (ROS2), or a string of numbers separated by
   * commas and/or spaces, optionally in brackets. Returns false if it is not.
   */
  static bool referenceHistogram(const rclcpp::Parameter& parameter, std::vector<double>& values)
  {
    switch (parameter.get_type())
    {
      case rclcpp::PARAMETER_NOT_SET:
        values.clear();
        return true;
      case rclcpp::PARAMETER_DOUBLE_ARRAY:
        values = parameter.as_double_array();
        return true;
#ifdef ROS2
      case rclcpp::PARAMETER_INTEGER_ARRAY:
        values.assign(parameter.as_integer_array().begin(), parameter.as_integer_array().end());
        return true;
#endif
      case rclcpp::PARAMETER_STRING:
      {
        std::string text = parameter.as_string();
        std::replace_if(text.begin(), text.end(), [](char c) { return c == '[' || c == ']' || c == ','; }, ' ');
        std::istringstream stream(text);
        stream.imbue(std::locale::classic());
        std::vector<double> parsed;
        std::string token;
        while (stream >> token)
        {
          // every token must be a finite number, e.g. "1e" and "1e9999" are rejected
          std::istringstream token_stream(token);
          token_stream.imbue(std::locale::classic());
          double value;
          if (!(token_stream >> value) || !(token_stream >> std::ws).eof() || !std::isfinite(value))
          {
            return false;
          }
          parsed.push_back(value);
        }
        values = parsed;
        return true;
      }
      default:
        return false;
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    std::vector<double> reference_histogram = reference_histogram_;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "reference_histogram" && !referenceHistogram(parameter, reference_histogram))
      {
        result.successful = false;
        result.reason = "reference_histogram must be a list of numbers";
        return result;
      }
    }
    reference_histogram_ = reference_histogram;
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "histogram_comparison_type")
        histogram_comparison_type_ = parameter.as_int();
      else if (parameter.get_name() == "histogram_size")
        histogram_size_ = parameter.as_int();
      else if (parameter.get_name() == "accumulate")
        accumulate_ = parameter.as_bool();
    }
    result.successful = true;
    return result;
  }

  static int compareMethod(int histogram_comparison_type)
  {
    switch (histogram_comparison_type)
    {
#if CV_MAJOR_VERSION >= 3
      case CompareHistogram_Chi_Square:
        return cv::HISTCMP_CHISQR;
      case CompareHistogram_Intersection:
        return cv::HISTCMP_INTERSECT;
      case CompareHistogram_Bhattacharyya:
        return cv::HISTCMP_BHATTACHARYYA;
      case CompareHistogram_Correlation:
      default:
        return cv::HISTCMP_CORREL;
#else  // cv::HISTCMP_* are available since OpenCV 3.0
      case CompareHistogram_Chi_Square:
        return CV_COMP_CHISQR;
      case CompareHistogram_Intersection:
        return CV_COMP_INTERSECT;
      case CompareHistogram_Bhattacharyya:
        return CV_COMP_BHATTACHARYYA;
      case CompareHistogram_Correlation:
      default:
        return CV_COMP_CORREL;
#endif
    }
  }

  /** @brief
   * Float64MultiArray of values in [channel] layout, or [channel][bin] layout if bins > 0
   */
  static std_msgs::msg::Float64MultiArray multiArray(const std::vector<double>& values, int channels, int bins = 0)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(bins > 0 ? 2 : 1);
    msg.layout.dim[0].label = "channel";
    msg.layout.dim[0].size = channels;
    msg.layout.dim[0].stride = values.size();
    if (bins > 0)
    {
      msg.layout.dim[1].label = "bin";
      msg.layout.dim[1].size = bins;
      msg.layout.dim[1].stride = bins;
    }
    msg.data = values;
    return msg;
  }

  // plot a histogram normalized to [0, 1]
  static void drawHistogram(cv::Mat& drawing, const cv::Mat& hist, const cv::Scalar& color, int thickness)
  {
    double bin_w = static_cast<double>(drawing.cols) / hist.rows;
    for (int i = 1; i < hist.rows; i++)
    {
      cv::line(drawing,
               cv::Point(cvRound(bin_w * (i - 1)), drawing.rows - cvRound(drawing.rows * hist.at<float>(i - 1, 0))),
               cv::Point(cvRound(bin_w * i), drawing.rows - cvRound(drawing.rows * hist.at<float>(i, 0))), color,
               thickness, 8, 0);
    }
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
    // Convert the image into BGR8, or MONO8 for single channel images (e.g. mono16)
    cv::Mat frame;
    try
    {
      bool mono = sensor_msgs::image_encodings::numChannels(msg->encoding) == 1 &&
                  !sensor_msgs::image_encodings::isBayer(msg->encoding);
      frame = cv_bridge::toCvShare(msg, mono ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8)
                  ->image;
    }
    catch (std::runtime_error& e)  // cv_bridge::Exception, or unknown encoding
    {
      RCLCPP_ERROR(get_logger(), "Failed to convert the image of encoding %s: %s", msg->encoding.c_str(), e.what());
      return;
    }
    catch (cv::Exception& e)  // e.g. cvtColor of an empty rgb8 image
    {
      RCLCPP_ERROR(get_logger(), "Failed to convert the image of encoding %s: %s", msg->encoding.c_str(), e.what());
      return;
    }
    if (frame.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Empty image");
      return;
    }

    try
    {
      std::lock_guard<std::mutex> lock(mutex_);
      int histogram_size = histogram_size_;

      //  Separate the image
      std::vector<cv::Mat> planes;
      cv::split(frame, planes);
      int channels = planes.size();

      //  Set the range
      float range[] = { 0, 256 };  // the upper boundary is exclusive
      const float* hist_range[] = { range };

      // Compute the histograms, accumulated only on the histograms of the same size
      bool accumulate = accumulate_ && hist_.size() == planes.size() && hist_[0].rows == histogram_size;
      if (!accumulate)
      {
        hist_.assign(channels, cv::Mat());
      }
      for (int j = 0; j < channels; j++)
      {
        int channel = 0;  // planes[j] has one channel
        cv::calcHist(&planes[j], 1, &channel, cv::Mat(), hist_[j], 1, &histogram_size, hist_range, true, accumulate);
      }

      // Normalize the histogram of each channel to [0, 1], into one (channels * histogram_size) x 1 histogram
      cv::Mat hist(channels * histogram_size, 1, CV_32F);
      for (int j = 0; j < channels; j++)
      {
        cv::Mat channel_hist = hist.rowRange(j * histogram_size, (j + 1) * histogram_size);
        cv::normalize(hist_[j], channel_hist, 0, 1, cv::NORM_MINMAX, CV_32F);
      }
      // NORM_MINMAX may give tiny negative values (e.g. -1e-10), which make Bhattacharyya distance NaN
      hist = cv::max(hist, 0.0);
      histogram_pub_->publish(
          multiArray(std::vector<double>(hist.begin<float>(), hist.end<float>()), channels, histogram_size));

      // Apply the histogram comparison methods
      std::string distance_text;
      cv::Mat reference;
      if (!reference_histogram_.empty())
      {
        if (reference_histogram_.size() != static_cast<size_t>(hist.rows))
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Size of reference_histogram (%zu) is not channels (%d) x histogram_size (%d)",
                               reference_histogram_.size(), channels, histogram_size);
        }
        else
        {
          reference.create(hist.rows, 1, CV_32F);
          for (int i = 0; i < hist.rows; i++)
          {
            // as hist, negative values make Bhattacharyya distance NaN
            reference.at<float>(i, 0) = std::max(0.0f, static_cast<float>(reference_histogram_[i]));
          }
          int compare_method = compareMethod(histogram_comparison_type_);
          std_msgs::msg::Float64 distance_msg;
          distance_msg.data = cv::compareHist(reference, hist, compare_method);
          distance_pub_->publish(distance_msg);
          distance_text = "distance: " + std::to_string(distance_msg.data);

          std::vector<double> channel_distances(channels);
          for (int j = 0; j < channels; j++)
          {
            cv::Range rows(j * histogram_size, (j + 1) * histogram_size);
            channel_distances[j] = cv::compareHist(reference.rowRange(rows), hist.rowRange(rows), compare_method);
          }
          channel_distances_pub_->publish(multiArray(channel_distances, channels));
        }
      }

      // Draw the histograms, and the reference histogram with thin dark lines
      cv::Mat drawing(400, 512, CV_8UC3, cv::Scalar(0, 0, 0));
      for (int j = 0; j < channels; j++)
      {
        // white for gray images, blue, green and red for BGR images
        cv::Scalar color = (channels == 1) ? cv::Scalar(255, 255, 255) :
                                             cv::Scalar(j == 0 ? 255 : 0, j == 1 ? 255 : 0, j == 2 ? 255 : 0);
        if (!reference.empty())
        {
          drawHistogram(drawing, reference.rowRange(j * histogram_size, (j + 1) * histogram_size), color * 0.5, 1);
        }
        drawHistogram(drawing, hist.rowRange(j * histogram_size, (j + 1) * histogram_size), color, 2);
      }

      if (!distance_text.empty())
      {
        cv::putText(drawing, distance_text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255),
                    2);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::imshow(window_name_, drawing);
        cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, drawing).toImageMsg();
      out_img->header.frame_id = input_frame_from_msg;
      img_pub_.publish(*out_img);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &CompareHistogramNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &CompareHistogramNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit CompareHistogramNodelet(const rclcpp::NodeOptions& options) : Nodelet("compare_histogram", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }

    window_name_ = "Histogram Window (" + std::string(get_fully_qualified_name()) + ")";

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    histogram_comparison_type_ = declare_parameter(
        "histogram_comparison_type", 0,
        paramDescriptor("Histogram Comparison method, 0: Correlation, 1: Chi_Square, 2: Intersection, 3: Bhattacharyya",
                        0, 3));
    histogram_size_ = declare_parameter("histogram_size", 256,
                                        paramDescriptor("Number of bins of the histogram of each channel", 1, 1024));
    accumulate_ = declare_parameter(
        "accumulate", false,
        paramDescriptor("Accumulation flag. If it is set, the histogram is not cleared between images"));
    rcl_interfaces::msg::ParameterDescriptor reference_descriptor =
        paramDescriptor("Reference of the histogram comparison, channels x histogram_size values in the layout of "
                        "~/histogram");
#ifdef ROS2
    // also accept integer[] (e.g. [0, 1, ...]) and string (e.g. "" or "[0.1, 1, ...]" from launch files)
    reference_descriptor.dynamic_typing = true;
    rclcpp::Parameter reference_parameter(
        "reference_histogram",
        declare_parameter("reference_histogram", rclcpp::ParameterValue(std::vector<double>()), reference_descriptor));
    if (!referenceHistogram(reference_parameter, reference_histogram_))
    {
      RCLCPP_ERROR(get_logger(), "reference_histogram must be a list of numbers: %s",
                   rclcpp::to_string(reference_parameter.get_parameter_value()).c_str());
    }
#else
    // ROS1 reads a list of numbers, integers or doubles, from the parameter server
    reference_histogram_ = declare_parameter("reference_histogram", std::vector<double>(), reference_descriptor);
#endif
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&CompareHistogramNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    histogram_pub_ = advertise<std_msgs::msg::Float64MultiArray>("~/histogram", 1);
    distance_pub_ = advertise<std_msgs::msg::Float64>("~/distance", 1);
    channel_distances_pub_ = advertise<std_msgs::msg::Float64MultiArray>("~/channel_distances", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::CompareHistogramNodelet)
