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

// https://github.com/isl-org/MiDaS

// https://github.com/opencv/opencv_zoo/tree/main/models/inpainting_lama
// https://github.com/opencv/opencv/blob/5.x/samples/dnn/inpainting.cpp
/**
 * Inpainting, fills the masked region of the image with LaMa of opencv_zoo, which needs OpenCV 5.0 or later. The
 * region is the rectangle given by ~set_roi service (opencv_apps/SetROI as the tracking nodes, e.g. by
 * scripts/set_image.py, an empty rectangle clears it) and the non-zero pixels of the latest image on mask (mono8,
 * e.g. ~/label of dnn_semantic_segmentation to remove the people). The image is resized to 512x512 for the model, and
 * only the masked region of the result is pasted back on the input image, which is published as it is when no
 * region is given
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"
#include <opencv_apps/srv/set_roi.hpp>

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
// LaMa (Apache 2.0) of https://github.com/opencv/opencv_zoo/tree/main/models/inpainting_lama, the commit is fixed
// not to change the file
const char* const LAMA_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
                                   "models/inpainting_lama/inpainting_lama_2025jan.onnx";
// the input size of LaMa
const int LAMA_INPUT_SIZE = 512;
}  // namespace

class DNNInpaintingNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_, mask_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Service<opencv_apps::srv::SetROI>::SharedPtr roi_srv_;

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
  // the model failed to be read, e.g. by OpenCV 4, not to read it again for each image until the parameters change
  bool model_failed_;
  // the region to fill: the rect of ~set_roi (empty if none) and the latest mask (empty if none)
  cv::Rect roi_;
  cv::Mat mask_;
  int mask_dilation_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "mask_dilation")
        mask_dilation_ = parameter.as_int();
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

  // (re)create the model with the parameters, or retry when it failed to be downloaded, called with mutex_
  bool updateModel()
  {
    if (!need_model_update_ && (!net_.empty() || model_failed_))
      return !net_.empty();
    need_model_update_ = false;
    model_failed_ = false;
    net_ = cv::dnn::Net();
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string model_file = modelFile(model_.empty() ? std::string(LAMA_MODEL_URL) : model_, get_logger());
    if (model_file.empty())
      return false;
    try
    {
      net_ = cv::dnn::readNet(model_file);
    }
    catch (cv::Exception& e)
    {
      // OpenCV 4 cannot read LaMa, e.g. DequantizeLinear by 4.6 and Mul by 4.12
      RCLCPP_ERROR(get_logger(), "Failed to read the model, which needs OpenCV 5.0 or later (this is %s): %s",
                   CV_VERSION, e.what());
      net_ = cv::dnn::Net();
      model_failed_ = true;
      return false;
    }
    net_.setPreferableBackend(backend);
    net_.setPreferableTarget(target);
    return true;
  }

  // the rect (x-y is the center) to fill, the image of the request is not used, as the rect is filled on each input
  // image
  void setROICb(const std::shared_ptr<opencv_apps::srv::SetROI::Request> req,
                std::shared_ptr<opencv_apps::srv::SetROI::Response> res)
  {
    cv::Rect roi(cvRound(req->rect.x - req->rect.width / 2), cvRound(req->rect.y - req->rect.height / 2),
                 cvRound(req->rect.width), cvRound(req->rect.height));
    std::lock_guard<std::mutex> lock(mutex_);
    // an empty rect clears the rect
    roi_ = roi.empty() ? cv::Rect() : roi;
    res->ok = true;
  }

  void maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try
    {
      cv::Mat mask = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
      std::lock_guard<std::mutex> lock(mutex_);
      mask_ = mask;
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to convert the mask image: %s", e.what());
    }
  }

  // the mask is a raw image, not of the image_transport of the input images, as the mask of match_template
  image_transport::Subscriber subscribeRawImage(const std::string& topic)
  {
    auto callback = std::bind(&DNNInpaintingNodelet::maskCallback, this, std::placeholders::_1);
#if __has_include(<image_transport/node_interfaces.hpp>)
    return image_transport::create_subscription(*this, topic, callback, "raw", subscriberQos(1));
#else
    return image_transport::create_subscription(this, topic, callback, "raw", subscriberQos(1).get_rmw_qos_profile());
#endif
  }

  // the inpainted image of LaMa (512x512) for the image and the mask (non-zero pixels to fill), called with mutex_
  cv::Mat inpaint(const cv::Mat& image, const cv::Mat& mask)
  {
    // the image in RGB in 0 - 1 as the original LaMa (the fill of the image in BGR, rgb: false of
    // samples/dnn/models.yml of OpenCV, is bluish), and the mask in 0 or 1 as the samples
    cv::Size size(LAMA_INPUT_SIZE, LAMA_INPUT_SIZE);
    cv::Mat image_blob = cv::dnn::blobFromImage(image, 1.0 / 255, size, cv::Scalar(), true, false, CV_32F);
    cv::Mat mask_blob = cv::dnn::blobFromImage(mask, 1.0, size, cv::Scalar(), false, false, CV_32F);
    mask_blob = (mask_blob > 0);
    mask_blob.convertTo(mask_blob, CV_32F, 1.0 / 255);
    net_.setInput(image_blob, "image");
    net_.setInput(mask_blob, "mask");
    cv::Mat output = net_.forward();  // 1 x 3 x 512 x 512, RGB in 0 - 255
    if (output.dims != 4 || output.size[1] != 3)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "The output of the model is not a color image");
      return cv::Mat();
    }
    std::vector<cv::Mat> channels;
    for (int i = 2; i >= 0; i--)  // BGR
      channels.push_back(cv::Mat(output.size[2], output.size[3], CV_32F, output.ptr<float>(0, i)));
    cv::Mat inpainted;
    cv::merge(channels, inpainted);
    inpainted.convertTo(inpainted, CV_8UC3);
    return inpainted;
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
      cv::Mat result = frame;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // the region to fill, the rect and the mask (resized to the image if it is not of its size)
        cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8U);
        mask(roi_ & cv::Rect(cv::Point(0, 0), frame.size())).setTo(255);
        if (!mask_.empty())
        {
          cv::Mat resized = mask_;
          if (mask_.size() != frame.size())
            cv::resize(mask_, resized, frame.size(), 0, 0, cv::INTER_NEAREST);
          mask.setTo(255, resized);
        }
        if (mask_dilation_ > 0)
          cv::dilate(mask, mask,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(2 * mask_dilation_ + 1, 2 * mask_dilation_ + 1)));
        if (cv::countNonZero(mask) == 0)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                               "No region to fill, call ~set_roi service or publish the mask image on mask");
        }
        else if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s",
                                model_.empty() ? LAMA_MODEL_URL : model_.c_str());
          return;
        }
        else
        {
          cv::Mat inpainted = inpaint(frame, mask);
          if (inpainted.empty())
            return;
          // only the masked region of the inpainted image, resized to the input image
          cv::resize(inpainted, inpainted, frame.size(), 0, 0, cv::INTER_CUBIC);
          result = frame.clone();
          inpainted.copyTo(result, mask);
        }
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Inpainting", result);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNInpaintingNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNInpaintingNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNInpaintingNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_inpainting", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    need_model_update_ = true;
    model_failed_ = false;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    model_ = declare_parameter(
        "model", std::string(""),
        paramDescriptor("Path or URL of the model (onnx) of the same input and output as LaMa, empty for LaMa of "
                        "opencv_zoo. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time"));
    // cpu by default, as OpenCV 5.1 runs LaMa on the CPU also with cuda, which is slower with the fallback
    backend_ = declare_parameter(
        "backend", std::string("cpu"),
        paramDescriptor(std::string(DNN_BACKEND_DESCRIPTION) +
                        " cpu by default, as OpenCV 5.1 runs LaMa on the CPU also with cuda, which is slower"));
    mask_dilation_ = declare_parameter(
        "mask_dilation", 10,
        paramDescriptor("Pixels to dilate the region to fill by, as the masks of segmentation (e.g. ~/label of "
                        "dnn_semantic_segmentation) miss the edges of the objects, which LaMa extends otherwise"));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNInpaintingNodelet::parametersCallback, this, std::placeholders::_1));

    // the latest mask is kept, even if the output topic is not subscribed
    mask_sub_ = subscribeRawImage("mask");
    // the region to fill may be of ~set_roi
    optionalInput("mask");
    img_pub_ = advertiseImage("~/image", 1);
    roi_srv_ = create_service<opencv_apps::srv::SetROI>(
        "~/set_roi", std::bind(&DNNInpaintingNodelet::setROICb, this, std::placeholders::_1, std::placeholders::_2));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNInpaintingNodelet)
