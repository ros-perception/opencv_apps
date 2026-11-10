/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Tal Regev.
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
// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
/**
 * This is a demo of Simple Example from wiki tutorial
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>
#include <string>

namespace opencv_apps
{
namespace simple_example
{
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  rclcpp::Node* node_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int queue_size_;
  bool debug_view_;

public:
  explicit ImageConverter(rclcpp::Node* node) : node_(node)
  {
    queue_size_ = node_->declare_parameter("queue_size", 1);
    debug_view_ = node_->declare_parameter("debug_view", false);

    // Subscrive to input video feed and publish output video feed
    // image_transport takes rclcpp::QoS since lyrical
#if __has_include(<image_transport/node_interfaces.hpp>)
    image_sub_ = image_transport::create_subscription(*node_, "image",
                                                      std::bind(&ImageConverter::imageCb, this, std::placeholders::_1),
                                                      "raw", rclcpp::SensorDataQoS().keep_last(queue_size_));
    image_pub_ = image_transport::create_publisher(*node_, "/image_converter/output_video/raw", rclcpp::QoS(1));
#else
    image_sub_ =
        image_transport::create_subscription(node_, "image",
                                             std::bind(&ImageConverter::imageCb, this, std::placeholders::_1), "raw",
                                             rclcpp::SensorDataQoS().keep_last(queue_size_).get_rmw_qos_profile());
    image_pub_ = image_transport::create_publisher(node_, "/image_converter/output_video/raw",
                                                   rclcpp::QoS(1).get_rmw_qos_profile());
#endif

    if (debug_view_)
    {
      cv::namedWindow(OPENCV_WINDOW);
    }
  }

  ~ImageConverter()
  {
    if (debug_view_)
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 110 && cv_ptr->image.cols > 110)
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2), 100, CV_RGB(255, 0, 0));

    if (debug_view_)
    {
      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);
    }

    // Output modified video stream
    image_pub_.publish(*cv_ptr->toImageMsg());
  }
};

}  // namespace simple_example

class SimpleExampleNodelet : public rclcpp::Node
{
  std::shared_ptr<simple_example::ImageConverter> ic_;

public:
  explicit SimpleExampleNodelet(const rclcpp::NodeOptions& options) : rclcpp::Node("simple_example", options)
  {
    ic_ = std::make_shared<simple_example::ImageConverter>(this);
  }
};

}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::SimpleExampleNodelet)
