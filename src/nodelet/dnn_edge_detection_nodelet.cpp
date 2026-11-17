// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Kei Okada.
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
 *   * Neither the name of the JSK Lab nor the names of its
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

// https://github.com/opencv/opencv/blob/5.x/samples/dnn/edge_detection.cpp
// https://github.com/opencv/opencv_zoo/tree/main/models/edge_detection_dexined
/**
 * Edge detection with DexiNed of opencv_zoo (MIT License), whose ~/image is the edges in mono8 as edge_detection,
 * the probability of the edge of each pixel (0 - 255). The model needs OpenCV 4.12 or later (OpenCV 4.6 and 4.10 cannot read
 * it).
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#define OPENCV_APPS_CV_VERSION_GE(major, minor)                                                                        \
  (CV_VERSION_MAJOR > (major) || (CV_VERSION_MAJOR == (major) && CV_VERSION_MINOR >= (minor)))

namespace opencv_apps
{
namespace
{
// DexiNed of opencv_zoo (MIT License), the commit is fixed not to change the file
const char* const DEXINED_MODEL_URL =
    "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
    "models/edge_detection_dexined/edge_detection_dexined_2024sep.onnx";
}  // namespace

class DNNEdgeDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string model_;
  std::string backend_;
  int input_width_;
  int input_height_;

  std::mutex mutex_;
  cv::dnn::Net net_;
  std::string output_name_;
  bool need_model_update_;
  bool read_failed_;  // the model could not be read, not retried until the parameters change

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "input_width")
        input_width_ = parameter.as_int();
      else if (parameter.get_name() == "input_height")
        input_height_ = parameter.as_int();
      else
      {
        if (parameter.get_name() == "model")
          model_ = parameter.as_string();
        else if (parameter.get_name() == "backend")
          backend_ = parameter.as_string();
        else
          continue;
        need_model_update_ = true;
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // (re)create the model with the parameters, or retry when it failed to load, called with mutex_
  bool updateModel()
  {
    if (!need_model_update_ && (!net_.empty() || read_failed_))
      return !read_failed_;
    need_model_update_ = false;
    read_failed_ = false;
    net_ = cv::dnn::Net();
#if !OPENCV_APPS_CV_VERSION_GE(4, 12)
    // cv::dnn of the older OpenCV cannot run the model (OpenCV 4.10 fails on DequantizeLinear)
    RCLCPP_ERROR(get_logger(), "DexiNed needs OpenCV 4.12 or later (this is %s)", CV_VERSION);
    read_failed_ = true;
    return false;
#endif
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string model_file = modelFile(model_.empty() ? std::string(DEXINED_MODEL_URL) : model_, get_logger());
    if (model_file.empty())
      return false;
    try
    {
      net_ = cv::dnn::readNet(model_file);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to read the model, which needs OpenCV 4.12 or later (this is %s): %s",
                   CV_VERSION, e.what());
      net_ = cv::dnn::Net();
      read_failed_ = true;
      return false;
    }
    net_.setPreferableBackend(backend);
    net_.setPreferableTarget(target);
    // the fused edges (block_cat) of DexiNed, whose other outputs (out_1 - out_6) are the edges of each block, or the
    // first output of the other models
    std::vector<std::string> names = net_.getUnconnectedOutLayersNames();
    output_name_ = names.empty() ? "" : names[0];
    if (std::find(names.begin(), names.end(), "block_cat") != names.end())
      output_name_ = "block_cat";
    return true;
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

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      if (frame.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Empty image is given");
        return;
      }

      // Do the work
      cv::Mat edges;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s", model_.c_str());
          return;
        }
        // the image in BGR minus the mean of BSDS500, as DexiNed of opencv_zoo
        net_.setInput(cv::dnn::blobFromImage(frame, 1.0, cv::Size(input_width_, input_height_),
                                             cv::Scalar(103.5, 116.2, 123.6), false, false, CV_32F));
        cv::Mat output = net_.forward(output_name_);  // 1 x 1 x height x width, the logits of the edges
        if (output.total() != static_cast<size_t>(output.size[output.dims - 2]) * output.size[output.dims - 1])
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "The output of the model is not an edge image");
          return;
        }
        cv::Mat logits(output.size[output.dims - 2], output.size[output.dims - 1], CV_32F, output.ptr<float>());
        // the probability of the edges by the sigmoid, not normalized by the range of the image as the sample, so
        // that an image without edges has no edges
        cv::Mat probability;
        cv::exp(-logits, probability);
        probability = 1.0 / (1.0 + probability);
        probability.convertTo(probability, CV_8U, 255);
        cv::resize(probability, edges, frame.size(), 0, 0, cv::INTER_LINEAR);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("DNN Edge Detection Demo", edges);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, edges).toImageMsg());
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

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNEdgeDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNEdgeDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNEdgeDetectionNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_edge_detection", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    need_model_update_ = true;
    read_failed_ = false;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    model_ = declare_parameter(
        "model", std::string(""),
        paramDescriptor("Path or URL of the model (onnx) of the same input and output as DexiNed, empty for DexiNed of "
                        "opencv_zoo (OpenCV 4.12 or later). The URL is downloaded to ~/.ros/opencv_apps/models/ at the "
                        "first time"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    input_width_ =
        declare_parameter("input_width", 512, paramDescriptor("Width of the network input, 512 for DexiNed", 1, 4096));
    input_height_ = declare_parameter("input_height", 512,
                                      paramDescriptor("Height of the network input, 512 for DexiNed", 1, 4096));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNEdgeDetectionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNEdgeDetectionNodelet)
