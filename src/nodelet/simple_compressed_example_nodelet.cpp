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
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace opencv_apps
{
namespace simple_compressed_example
{
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  rclcpp::Node* node_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  int queue_size_;
  bool debug_view_;

public:
  explicit ImageConverter(rclcpp::Node* node) : node_(node)
  {
    queue_size_ = node_->declare_parameter("queue_size", 3);
    debug_view_ = node_->declare_parameter("debug_view", false);

    // Subscrive to input video feed and publish output video feed
    image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", rclcpp::SensorDataQoS().keep_last(queue_size_),
        std::bind(&ImageConverter::imageCb, this, std::placeholders::_1));
    image_pub_ =
        node_->create_publisher<sensor_msgs::msg::CompressedImage>("/image_converter/output_video/compressed", 1);

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

  void imageCb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
/*
Supporting CompressedImage in cv_bridge has been started from 1.11.9 (2015-11-29)
note : hydro 1.10.18, indigo : 1.11.13 ...
https://github.com/ros-perception/vision_opencv/pull/70
 */
#ifndef CV_BRIDGE_COMPRESSED_IMAGE_IS_NOT_SUPPORTED
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
#else
      // cv_ptr = cv_bridge::toCvCopyImpl(matFromImage(msg), msg.header, sensor_msgs::image_encodings::BGR8,
      // sensor_msgs::image_encodings::BGR8);
      // cv::Mat matFromImage(const sensor_msgs::CompressedImage& source)
      cv::Mat jpegData(1, msg->data.size(), CV_8UC1);
      jpegData.data = const_cast<uchar*>(&msg->data[0]);
      cv::InputArray data(jpegData);
      cv::Mat image = cv::imdecode(data, cv::IMREAD_COLOR);

      // cv_ptr = cv_bridge::toCvCopyImpl(bgrMat, msg->header, sensor_msgs::image_encodings::BGR8,
      // sensor_msgs::image_encodings::BGR8);
      sensor_msgs::msg::Image ros_image;
      ros_image.header = msg->header;
      ros_image.height = image.rows;
      ros_image.width = image.cols;
      ros_image.encoding = sensor_msgs::image_encodings::BGR8;
      ros_image.is_bigendian = false;
      ros_image.step = image.cols * image.elemSize();
      size_t size = ros_image.step * image.rows;
      ros_image.data.resize(size);

      if (image.isContinuous())
      {
        memcpy((char*)(&ros_image.data[0]), image.data, size);
      }
      else
      {
        // Copy by row by row
        uchar* ros_data_ptr = (uchar*)(&ros_image.data[0]);
        uchar* cv_data_ptr = image.data;
        for (int i = 0; i < image.rows; ++i)
        {
          memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
          ros_data_ptr += ros_image.step;
          cv_data_ptr += image.step;
        }
      }
      cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
#endif
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
#ifndef CV_BRIDGE_COMPRESSED_IMAGE_IS_NOT_SUPPORTED
    std::unique_ptr<sensor_msgs::msg::CompressedImage> out_msg(new sensor_msgs::msg::CompressedImage());
    cv_ptr->toCompressedImageMsg(*out_msg);
    image_pub_->publish(std::move(out_msg));
#else
    image_pub_->publish(toCompressedImageMsg(cv_ptr));
#endif
  }
#ifdef CV_BRIDGE_COMPRESSED_IMAGE_IS_NOT_SUPPORTED
  sensor_msgs::msg::CompressedImage toCompressedImageMsg(cv_bridge::CvImagePtr cv_ptr) const
  {
    sensor_msgs::msg::CompressedImage ros_image;
    const std::string dst_format = std::string();
    ros_image.header = cv_ptr->header;
    cv::Mat image;
    if (cv_ptr->encoding != sensor_msgs::image_encodings::BGR8)
    {
      cv_bridge::CvImagePtr tempThis = std::make_shared<cv_bridge::CvImage>(*cv_ptr);
      cv_bridge::CvImagePtr temp = cv_bridge::cvtColor(tempThis, sensor_msgs::image_encodings::BGR8);
      cv_ptr->image = temp->image;
    }
    else
    {
      image = cv_ptr->image;
    }
    std::vector<uchar> buf;
    if (dst_format.empty() || dst_format == "jpg")
    {
      ros_image.format = "jpg";
      cv::imencode(".jpg", image, buf);
    }

    if (dst_format == "png")
    {
      ros_image.format = "png";
      cv::imencode(".png", image, buf);
    }

    // TODO: check this formats (on rviz) and add more formats
    // from http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html#Mat imread(const
    // string& filename, int flags)
    if (dst_format == "jp2")
    {
      ros_image.format = "jp2";
      cv::imencode(".jp2", image, buf);
    }

    if (dst_format == "bmp")
    {
      ros_image.format = "bmp";
      cv::imencode(".bmp", image, buf);
    }

    if (dst_format == "tif")
    {
      ros_image.format = "tif";
      cv::imencode(".tif", image, buf);
    }

    ros_image.data = buf;
    return ros_image;
  }
#endif
};

}  // namespace simple_compressed_example

class SimpleCompressedExampleNodelet : public rclcpp::Node
{
  std::shared_ptr<simple_compressed_example::ImageConverter> ic_;

public:
  explicit SimpleCompressedExampleNodelet(const rclcpp::NodeOptions& options)
    : rclcpp::Node("simple_compressed_example", options)
  {
    ic_ = std::make_shared<simple_compressed_example::ImageConverter>(this);
  }
};

}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::SimpleCompressedExampleNodelet)
