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

// https://github.com/opencv/opencv/blob/4.x/samples/dnn/object_detection.cpp
/**
 * Object detection with a DNN model of cv::dnn::DetectionModel, e.g. YOLO (darknet), SSD (caffe, tensorflow), ...
 * The default is YOLOv4-tiny trained on COCO, https://github.com/AlexeyAB/darknet
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/detection.hpp>
#include <opencv_apps/msg/detection_array_stamped.hpp>

#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
class DNNObjectDetectionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::DetectionArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string model_;
  std::string config_;
  std::string classes_;
  std::string backend_;
  int input_width_;
  int input_height_;
  double scale_;
  bool swap_rb_;
  double confidence_threshold_;
  double nms_threshold_;

  std::mutex mutex_;
  std::unique_ptr<cv::dnn::DetectionModel> model_ptr_;
  std::vector<std::string> class_names_;
  bool need_model_update_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "confidence_threshold")
        confidence_threshold_ = parameter.as_double();
      else if (parameter.get_name() == "nms_threshold")
        nms_threshold_ = parameter.as_double();
      else
      {
        if (parameter.get_name() == "model")
          model_ = parameter.as_string();
        else if (parameter.get_name() == "config")
          config_ = parameter.as_string();
        else if (parameter.get_name() == "classes")
          classes_ = parameter.as_string();
        else if (parameter.get_name() == "backend")
          backend_ = parameter.as_string();
        else if (parameter.get_name() == "input_width")
          input_width_ = parameter.as_int();
        else if (parameter.get_name() == "input_height")
          input_height_ = parameter.as_int();
        else if (parameter.get_name() == "scale")
          scale_ = parameter.as_double();
        else if (parameter.get_name() == "swap_rb")
          swap_rb_ = parameter.as_bool();
        else
          continue;
        need_model_update_ = true;
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // (re)create the model with the parameters, called with mutex_
  bool updateModel()
  {
    if (!need_model_update_)
      return static_cast<bool>(model_ptr_);
    need_model_update_ = false;
    model_ptr_.reset();
    class_names_.clear();
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    // YOLOv4-tiny trained on COCO when the model is not given
    const std::string darknet = "https://raw.githubusercontent.com/AlexeyAB/darknet/"
                                "8a0bf84c19e38214219dbd3345f04ce778426c57/";
    std::string model = model_.empty() ? "https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/"
                                         "yolov4-tiny.weights" :
                                         model_;
    std::string config = model_.empty() ? darknet + "cfg/yolov4-tiny.cfg" : config_;
    std::string classes = model_.empty() ? darknet + "data/coco.names" : classes_;
    std::string model_file = modelFile(model, get_logger());
    std::string config_file = config.empty() ? std::string() : modelFile(config, get_logger());
    if (model_file.empty() || (!config.empty() && config_file.empty()))
      return false;
    if (!classes.empty())
    {
      std::string classes_file = modelFile(classes, get_logger());
      std::ifstream ifs(classes_file);
      if (classes_file.empty() || !ifs)
        RCLCPP_ERROR(get_logger(), "Failed to read the class names %s, use the class ids", classes.c_str());
      std::string line;
      while (std::getline(ifs, line))
      {
        if (!line.empty() && line.back() == '\r')
          line.pop_back();
        class_names_.push_back(line);
      }
    }
    model_ptr_.reset(new cv::dnn::DetectionModel(model_file, config_file));
    model_ptr_->setPreferableBackend(static_cast<cv::dnn::Backend>(backend));
    model_ptr_->setPreferableTarget(static_cast<cv::dnn::Target>(target));
    model_ptr_->setInputParams(scale_, cv::Size(input_width_, input_height_), cv::Scalar(), swap_rb_);
    return true;
  }

  std::string className(int class_id) const
  {
    if (class_id >= 0 && class_id < static_cast<int>(class_names_.size()))
      return class_names_[class_id];
    return std::to_string(class_id);
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

      // Messages
      opencv_apps::msg::DetectionArrayStamped detections_msg;
      detections_msg.header = msg->header;

      // Do the work
      std::vector<int> class_ids;
      std::vector<float> confidences;
      std::vector<cv::Rect> boxes;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s", model_.c_str());
          return;
        }
        model_ptr_->detect(frame, class_ids, confidences, boxes, static_cast<float>(confidence_threshold_),
                           static_cast<float>(nms_threshold_));
        for (size_t i = 0; i < boxes.size(); i++)
        {
          const cv::Rect& box = boxes[i];
          opencv_apps::msg::Detection detection_msg;
          detection_msg.label = className(class_ids[i]);
          detection_msg.id = class_ids[i];
          detection_msg.score = confidences[i];
          detection_msg.rect.x = box.x + box.width / 2.0;
          detection_msg.rect.y = box.y + box.height / 2.0;
          detection_msg.rect.width = box.width;
          detection_msg.rect.height = box.height;
          detections_msg.detections.push_back(detection_msg);

          cv::Scalar color =
              cv::Scalar((class_ids[i] * 67) % 256, (class_ids[i] * 131 + 100) % 256, (class_ids[i] * 197 + 50) % 256);
          cv::rectangle(frame, box, color, 2);
          std::string text = cv::format("%s %.2f", detection_msg.label.c_str(), confidences[i]);
          int baseline;
          cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
          cv::Point origin(box.x, std::max(box.y, text_size.height));
          cv::rectangle(frame, cv::Rect(origin.x, origin.y - text_size.height, text_size.width, text_size.height),
                        color, cv::FILLED);
          cv::putText(frame, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Object detection", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(detections_msg);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNObjectDetectionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNObjectDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNObjectDetectionNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_object_detection", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    need_model_update_ = true;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    model_ = declare_parameter(
        "model", std::string(""),
        paramDescriptor("Path or URL of the model (e.g. .weights, .onnx, .caffemodel, .pb), empty for YOLOv4-tiny "
                        "trained on COCO. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time"));
    config_ = declare_parameter("config", std::string(""),
                                paramDescriptor("Path or URL of the network configuration (e.g. .cfg, .prototxt, "
                                                ".pbtxt) of the model, or empty"));
    classes_ = declare_parameter("classes", std::string(""),
                                 paramDescriptor("Path or URL of the class names of the model, one name per line, or "
                                                 "empty"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    input_width_ = declare_parameter("input_width", 416, paramDescriptor("Width of the network input", 1, 4096));
    input_height_ = declare_parameter("input_height", 416, paramDescriptor("Height of the network input", 1, 4096));
    scale_ = declare_parameter("scale", 1.0 / 255,
                               paramDescriptor("Multiplier of the pixel values for the network input", 0.0, 1.0));
    swap_rb_ = declare_parameter("swap_rb", true,
                                 paramDescriptor("Swap the red and blue channels, true for the models trained in RGB"));
    confidence_threshold_ = declare_parameter(
        "confidence_threshold", 0.5, paramDescriptor("Threshold of the confidence to detect the objects", 0.0, 1.0));
    nms_threshold_ = declare_parameter(
        "nms_threshold", 0.4,
        paramDescriptor("Threshold of the overlap of the objects to suppress, by non maximum suppression", 0.0, 1.0));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNObjectDetectionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::DetectionArrayStamped>("~/detections", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNObjectDetectionNodelet)
