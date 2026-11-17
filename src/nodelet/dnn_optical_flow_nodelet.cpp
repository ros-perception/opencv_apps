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

// https://github.com/opencv/opencv_zoo/tree/main/models/optical_flow_estimation_raft
/**
 * Dense optical flow between the consecutive images by RAFT of opencv_zoo (trained on Sintel, 360x480 input; the model
 * is under the BSD-3-Clause license and its conversion to onnx by PINTO0309 under the MIT License), with the same
 * outputs as fback_flow, so that it can replace fback_flow: ~/flows sampled on a grid of 16 pixels and ~/image of the
 * flows drawn on the previous image. The model needs OpenCV 4.10 or later (OpenCV 4.6 cannot read it), and runs on
 * the CPU with OpenCV 4.x, whose CUDA backend crashes with it: about 1.3 s per image with OpenCV 4.12 and 0.4 s with
 * OpenCV 5.1 on a desktop CPU, so the flows are of the images which the node processes, not of all the images.
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/flow_array_stamped.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
namespace
{
// RAFT trained on Sintel (BSD-3-Clause, converted to onnx by PINTO0309, MIT License), 360 x 480 input, of opencv_zoo
const char* const RAFT_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
                                   "models/optical_flow_estimation_raft/optical_flow_estimation_raft_2023aug.onnx";
}  // namespace

class DNNOpticalFlowNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::FlowArrayStamped>::SharedPtr msg_pub_;

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
  bool need_model_update_;
  bool read_failed_;  // the model could not be read, not retried until the parameters change

  // the previous image in gray, and its network input
  cv::Mat prevgray_, prev_input_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else
      {
        if (parameter.get_name() == "model")
          model_ = parameter.as_string();
        else if (parameter.get_name() == "backend")
          backend_ = parameter.as_string();
        else if (parameter.get_name() == "input_width")
          input_width_ = parameter.as_int();
        else if (parameter.get_name() == "input_height")
          input_height_ = parameter.as_int();
        else
          continue;
        need_model_update_ = true;
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // (re)create the model with the parameters, or retry when it failed to download, called with mutex_
  bool updateModel()
  {
    if (!need_model_update_ && (!net_.empty() || read_failed_))
      return !read_failed_;
    need_model_update_ = false;
    read_failed_ = false;
    net_ = cv::dnn::Net();
    prev_input_ = cv::Mat();  // the input of the previous image may be of the other size
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
#if CV_VERSION_MAJOR < 5
    // RAFT crashes in the CUDA backend of OpenCV 4.x (4.12), which aborts the process instead of throwing
    if (backend == cv::dnn::DNN_BACKEND_CUDA)
    {
      RCLCPP_WARN(get_logger(), "RAFT crashes with the CUDA backend of OpenCV %s, runs on the CPU instead", CV_VERSION);
      backend = cv::dnn::DNN_BACKEND_OPENCV;
      target = cv::dnn::DNN_TARGET_CPU;
    }
#endif
    std::string model_file = modelFile(model_.empty() ? std::string(RAFT_MODEL_URL) : model_, get_logger());
    if (model_file.empty())
      return false;
    try
    {
      net_ = cv::dnn::readNet(model_file);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to read the model, which needs OpenCV 4.10 or later (this is %s): %s",
                   CV_VERSION, e.what());
      net_ = cv::dnn::Net();
      read_failed_ = true;
      return false;
    }
    net_.setPreferableBackend(backend);
    net_.setPreferableTarget(target);
    return true;
  }

  // the network input of the image: RGB, 0 - 255, resized to the input size, as raft.py of opencv_zoo
  cv::Mat networkInput(const cv::Mat& frame)
  {
    return cv::dnn::blobFromImage(frame, 1.0, cv::Size(input_width_, input_height_), cv::Scalar(), true, false);
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

      // Messages
      opencv_apps::msg::FlowArrayStamped flows_msg;
      flows_msg.header = msg->header;

      // Check if clock is jumped back
      if (get_parameter("use_sim_time").as_bool() && prev_stamp_ > rclcpp::Time(msg->header.stamp))
      {
        RCLCPP_WARN_STREAM(get_logger(), "Detected jump back in time of " << rclcpp::Time(msg->header.stamp).seconds()
                                                                          << ". Clearing optical flow cache.");
        prevgray_ = cv::Mat();
      }
      // the flow of the images of the different sizes is not computed
      if (!prevgray_.empty() && prevgray_.size() != frame.size())
        prevgray_ = cv::Mat();

      // Do the work: the flow (in the pixels of the image) from the previous image to this image
      cv::Mat gray, flow;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s", model_.c_str());
          return;
        }
        cv::Mat input = networkInput(frame);
        if (!prevgray_.empty() && !prev_input_.empty())
        {
          net_.setInput(prev_input_, "0");
          net_.setInput(input, "1");
          // the flow of the input size (1 x 2 x height x width), and of 1/8 of it, take the larger one
          std::vector<cv::Mat> outputs;
          net_.forward(outputs, net_.getUnconnectedOutLayersNames());
          const cv::Mat* output = nullptr;
          for (const cv::Mat& o : outputs)
            if (o.dims == 4 && o.size[1] == 2 && (output == nullptr || o.total() > output->total()))
              output = &o;
          if (output == nullptr)
          {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "The output of the model is not a flow");
            return;
          }
          int height = output->size[2], width = output->size[3];
          cv::Mat fx(height, width, CV_32F, const_cast<float*>(output->ptr<float>(0, 0)));
          cv::Mat fy(height, width, CV_32F, const_cast<float*>(output->ptr<float>(0, 1)));
          // the flow in the pixels of the image, from the pixels of the output
          cv::Mat channels[2];
          cv::resize(fx, channels[0], frame.size(), 0, 0, cv::INTER_LINEAR);
          cv::resize(fy, channels[1], frame.size(), 0, 0, cv::INTER_LINEAR);
          channels[0] *= static_cast<double>(frame.cols) / width;
          channels[1] *= static_cast<double>(frame.rows) / height;
          cv::merge(channels, 2, flow);
        }
        prev_input_ = input;
      }

      // the flows on the previous image and in the message, as fback_flow, or no flows on the first image
      cv::Mat cflow;
      cv::cvtColor(flow.empty() ? gray : prevgray_, cflow, cv::COLOR_GRAY2BGR);
      if (!flow.empty())
      {
        int step = 16;
        cv::Scalar color = cv::Scalar(0, 255, 0);
        for (int y = 0; y < cflow.rows; y += step)
          for (int x = 0; x < cflow.cols; x += step)
          {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            cv::line(cflow, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
            cv::circle(cflow, cv::Point(x, y), 2, color, -1);

            opencv_apps::msg::Flow flow_msg;
            flow_msg.point.x = x;
            flow_msg.point.y = y;
            flow_msg.velocity.x = fxy.x;
            flow_msg.velocity.y = fxy.y;
            flows_msg.status.push_back(true);
            flows_msg.flow.push_back(flow_msg);
          }
      }
      prevgray_ = gray;

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Optical flow", cflow);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cflow).toImageMsg());
      msg_pub_->publish(flows_msg);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNOpticalFlowNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNOpticalFlowNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNOpticalFlowNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_optical_flow", options)
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
        paramDescriptor("Path or URL of the RAFT model (onnx) of the inputs 0 and 1 (the previous and the current "
                        "image) and the output of the flow, empty for RAFT of opencv_zoo (OpenCV 4.10 or later). The "
                        "URL is downloaded to ~/.ros/opencv_apps/models/ at the first time"));
    backend_ =
        declare_parameter("backend", std::string("auto"),
                          paramDescriptor(std::string(DNN_BACKEND_DESCRIPTION) +
                                          " RAFT runs on the CPU instead of CUDA with OpenCV 4.x, which crashes"));
    input_width_ = declare_parameter(
        "input_width", 480, paramDescriptor("Width of the network input, 480 for RAFT of opencv_zoo", 1, 4096));
    input_height_ = declare_parameter(
        "input_height", 360, paramDescriptor("Height of the network input, 360 for RAFT of opencv_zoo", 1, 4096));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNOpticalFlowNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::FlowArrayStamped>("~/flows", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNOpticalFlowNodelet)
