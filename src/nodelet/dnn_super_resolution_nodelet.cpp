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

// https://github.com/opencv/opencv_contrib/tree/5.x/modules/dnn_superres
/**
 * Super resolution of the image, by default with ESPCN x2, or with FSRCNN x2 or x3, the onnx models of dnn_superres
 * of opencv_contrib (in opencv_extra), converted from the TensorFlow models of TF-ESPCN and FSRCNN_Tensorflow
 * (Apache-2.0). As dnn_superres, the model enlarges the Y channel of YCrCb (0 - 1), and the Cr and Cb channels are
 * enlarged by the bicubic interpolation. The scale (the size of ~image / the size of the input) is that of the
 * model. The models need OpenCV 4.10 or later (OpenCV 4.6 cannot read them).
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
namespace
{
// ESPCN x2 of dnn_superres in opencv_extra, the commit is fixed not to change the file
const char* const ESPCN_MODEL_URL = "https://raw.githubusercontent.com/opencv/opencv_extra/"
                                    "697e5c3de828f3154e82f222dbe902d622bbe19a/testdata/cv/dnn_superres/ESPCN_x2.onnx";
}  // namespace

class DNNSuperResolutionNodelet : public opencv_apps::Nodelet
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

  std::mutex mutex_;
  cv::dnn::Net net_;
  bool need_model_update_;
  bool read_failed_;  // the model could not be read, not retried until the parameters change

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
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string model_file = modelFile(model_.empty() ? std::string(ESPCN_MODEL_URL) : model_, get_logger());
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
      cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      if (frame.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Empty image is given");
        return;
      }

      // Do the work
      cv::Mat ycrcb;
      cv::cvtColor(frame, ycrcb, cv::COLOR_BGR2YCrCb);
      std::vector<cv::Mat> channels;
      cv::split(ycrcb, channels);
      cv::Mat y_output;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s", model_.c_str());
          return;
        }
        // the Y channel in 0 - 1, in NHWC (1 x height x width x 1), as dnn_superres
        cv::Mat y;
        channels[0].convertTo(y, CV_32F, 1.0 / 255);
        const int shape[] = { 1, y.rows, y.cols, 1 };
        net_.setInput(cv::Mat(4, shape, CV_32F, y.ptr<float>()));
        cv::Mat output = net_.forward();  // 1 x 1 x (scale * height) x (scale * width)
        if (output.dims != 4 || output.size[0] != 1 || output.size[1] != 1 || output.size[2] % y.rows != 0 ||
            output.size[2] / y.rows != output.size[3] / y.cols || output.size[3] % y.cols != 0)
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                                "The output of the model is not the Y channel enlarged by an integer scale");
          return;
        }
        cv::Mat(output.size[2], output.size[3], CV_32F, output.ptr<float>()).convertTo(y_output, CV_8U, 255.0);
      }

      // the Cr and Cb channels by the bicubic interpolation, with the Y channel of the model
      for (int i = 1; i < 3; i++)
        cv::resize(channels[i], channels[i], y_output.size(), 0, 0, cv::INTER_CUBIC);
      channels[0] = y_output;
      cv::Mat result;
      cv::merge(channels, result);
      cv::cvtColor(result, result, cv::COLOR_YCrCb2BGR);

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Super resolution", result);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, result).toImageMsg());
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNSuperResolutionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNSuperResolutionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNSuperResolutionNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_super_resolution", options)
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
        paramDescriptor("Path or URL of the model (onnx) of dnn_superres, ESPCN or FSRCNN, whose scale is the scale "
                        "of ~image, empty for ESPCN x2. The URL is downloaded to ~/.ros/opencv_apps/models/ at the "
                        "first time"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNSuperResolutionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNSuperResolutionNodelet)
