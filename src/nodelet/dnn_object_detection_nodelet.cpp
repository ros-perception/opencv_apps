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
// https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox
/**
 * Object detection with YOLOX (onnx), by default YOLOX-S trained on COCO of opencv_zoo, or with a model of
 * cv::dnn::DetectionModel, e.g. YOLO (darknet), SSD (caffe, tensorflow), which OpenCV 5 cannot read except onnx
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/detection.hpp>
#include <opencv_apps/msg/detection_array_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
namespace
{
// YOLOX-S trained on COCO, https://github.com/opencv/opencv_zoo, the commit is fixed not to change the file
const char* const YOLOX_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
                                    "models/object_detection_yolox/object_detection_yolox_2022nov.onnx";

// the 80 classes of COCO, in the order of the YOLOX models
const char* const COCO_CLASSES[] = {
  "person",         "bicycle",    "car",           "motorcycle",    "airplane",     "bus",           "train",
  "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",    "parking meter", "bench",
  "bird",           "cat",        "dog",           "horse",         "sheep",        "cow",           "elephant",
  "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",     "handbag",       "tie",
  "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball",  "kite",          "baseball bat",
  "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",       "wine glass",    "cup",
  "fork",           "knife",      "spoon",         "bowl",          "banana",       "apple",         "sandwich",
  "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",        "donut",         "cake",
  "chair",          "couch",      "potted plant",  "bed",           "dining table", "toilet",        "tv",
  "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",   "microwave",     "oven",
  "toaster",        "sink",       "refrigerator",  "book",          "clock",        "vase",          "scissors",
  "teddy bear",     "hair drier", "toothbrush"
};
}  // namespace

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

  std::string model_type_;
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
  cv::dnn::Net yolox_;                                  // model_type yolox
  std::unique_ptr<cv::dnn::DetectionModel> model_ptr_;  // model_type detection_model
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
        if (parameter.get_name() == "model_type")
          model_type_ = parameter.as_string();
        else if (parameter.get_name() == "model")
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

  // (re)create the model with the parameters, or retry when it failed to load, called with mutex_
  bool updateModel()
  {
    if (!need_model_update_ && (model_ptr_ || !yolox_.empty()))
      return true;
    need_model_update_ = false;
    yolox_ = cv::dnn::Net();
    model_ptr_.reset();
    class_names_.clear();
    if (model_type_ != "yolox" && model_type_ != "detection_model")
    {
      RCLCPP_ERROR(get_logger(), "Unknown model_type %s, use yolox or detection_model", model_type_.c_str());
      return false;
    }
    if (model_type_ == "detection_model" && model_.empty())
    {
      RCLCPP_ERROR(get_logger(), "model is required for model_type detection_model");
      return false;
    }
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string model_file = modelFile(model_.empty() ? std::string(YOLOX_MODEL_URL) : model_, get_logger());
    std::string config_file = config_.empty() ? std::string() : modelFile(config_, get_logger());
    if (model_file.empty() || (!config_.empty() && config_file.empty()))
      return false;
    if (!classes_.empty())
    {
      std::string classes_file = modelFile(classes_, get_logger());
      std::ifstream ifs(classes_file);
      if (classes_file.empty() || !ifs)
        RCLCPP_ERROR(get_logger(), "Failed to read the class names %s, use the class ids", classes_.c_str());
      std::string line;
      while (std::getline(ifs, line))
      {
        if (!line.empty() && line.back() == '\r')
          line.pop_back();
        class_names_.push_back(line);
      }
    }
    else if (model_type_ == "yolox")
    {
      class_names_.assign(std::begin(COCO_CLASSES), std::end(COCO_CLASSES));
    }
    if (model_type_ == "yolox")
    {
      yolox_ = cv::dnn::readNet(model_file, config_file);
      yolox_.setPreferableBackend(backend);
      yolox_.setPreferableTarget(target);
    }
    else
    {
      model_ptr_.reset(new cv::dnn::DetectionModel(model_file, config_file));
      model_ptr_->setPreferableBackend(static_cast<cv::dnn::Backend>(backend));
      model_ptr_->setPreferableTarget(static_cast<cv::dnn::Target>(target));
      model_ptr_->setInputParams(scale_, cv::Size(input_width_, input_height_), cv::Scalar(), swap_rb_);
    }
    return true;
  }

  // detect the objects by YOLOX, as object_detection_yolox of opencv_zoo, called with mutex_
  void detectYOLOX(const cv::Mat& frame, std::vector<int>& class_ids, std::vector<float>& confidences,
                   std::vector<cv::Rect>& boxes)
  {
    // the image in RGB, resized keeping the aspect ratio into the top-left of the input, padded with 114
    cv::Size input_size(input_width_, input_height_);
    double ratio = std::min(static_cast<double>(input_size.width) / frame.cols,
                            static_cast<double>(input_size.height) / frame.rows);
    cv::Size resized_size(std::max(1, static_cast<int>(frame.cols * ratio)),
                          std::max(1, static_cast<int>(frame.rows * ratio)));
    cv::Mat input(input_size, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat resized = input(cv::Rect(cv::Point(0, 0), resized_size));
    cv::resize(frame, resized, resized_size);
    yolox_.setInput(cv::dnn::blobFromImage(input, 1.0, cv::Size(), cv::Scalar(), true, false));
    cv::Mat output = yolox_.forward();  // 1 x N x (4 box + 1 objectness + classes)
    if (output.dims != 3 || output.size[2] <= 5)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "The output of the model is not of YOLOX");
      return;
    }
    cv::Mat predictions(output.size[1], output.size[2], CV_32F, output.ptr<float>());

    // the predictions are on the grids of the strides 8, 16 and 32, in the row major order
    std::vector<int> grid_class_ids;
    std::vector<float> grid_confidences;
    std::vector<cv::Rect> grid_boxes;
    int row = 0;
    for (int stride : { 8, 16, 32 })
    {
      for (int y = 0; y < input_size.height / stride; y++)
      {
        for (int x = 0; x < input_size.width / stride && row < predictions.rows; x++, row++)
        {
          const float* p = predictions.ptr<float>(row);
          cv::Point class_id;
          double class_score;
          cv::minMaxLoc(cv::Mat(1, predictions.cols - 5, CV_32F, const_cast<float*>(p + 5)), nullptr, &class_score,
                        nullptr, &class_id);
          float confidence = p[4] * static_cast<float>(class_score);
          if (confidence < confidence_threshold_)
            continue;
          double cx = (p[0] + x) * stride / ratio;
          double cy = (p[1] + y) * stride / ratio;
          double width = std::exp(p[2]) * stride / ratio;
          double height = std::exp(p[3]) * stride / ratio;
          grid_class_ids.push_back(class_id.x);
          grid_confidences.push_back(confidence);
          grid_boxes.push_back(
              cv::Rect(cvRound(cx - width / 2), cvRound(cy - height / 2), cvRound(width), cvRound(height)));
        }
      }
    }
    if (row != predictions.rows)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                            "The output of the model (%d) does not match the input size %dx%d", predictions.rows,
                            input_size.width, input_size.height);
      return;
    }

    // non maximum suppression for each class
    std::map<int, std::vector<int> > class_candidates;
    for (size_t i = 0; i < grid_boxes.size(); i++)
      class_candidates[grid_class_ids[i]].push_back(static_cast<int>(i));
    for (const std::pair<const int, std::vector<int> >& candidates : class_candidates)
    {
      std::vector<cv::Rect> class_boxes;
      std::vector<float> class_confidences;
      for (int i : candidates.second)
      {
        class_boxes.push_back(grid_boxes[i]);
        class_confidences.push_back(grid_confidences[i]);
      }
      std::vector<int> indices;
      cv::dnn::NMSBoxes(class_boxes, class_confidences, static_cast<float>(confidence_threshold_),
                        static_cast<float>(nms_threshold_), indices);
      for (int index : indices)
      {
        int i = candidates.second[index];
        class_ids.push_back(grid_class_ids[i]);
        confidences.push_back(grid_confidences[i]);
        boxes.push_back(grid_boxes[i]);
      }
    }
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
        if (model_ptr_)
          model_ptr_->detect(frame, class_ids, confidences, boxes, static_cast<float>(confidence_threshold_),
                             static_cast<float>(nms_threshold_));
        else
          detectYOLOX(frame, class_ids, confidences, boxes);
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
    model_type_ = declare_parameter(
        "model_type", std::string("yolox"),
        paramDescriptor("yolox for the YOLOX models (onnx), or detection_model for the models of "
                        "cv::dnn::DetectionModel, e.g. YOLO (darknet), SSD (caffe, tensorflow), with config, "
                        "input_width, input_height, scale and swap_rb"));
    model_ = declare_parameter(
        "model", std::string(""),
        paramDescriptor("Path or URL of the model (e.g. .onnx, .weights, .caffemodel, .pb), empty for YOLOX-S "
                        "trained on COCO of opencv_zoo. The URL is downloaded to ~/.ros/opencv_apps/models/ at the "
                        "first time"));
    config_ = declare_parameter("config", std::string(""),
                                paramDescriptor("Path or URL of the network configuration (e.g. .cfg, .prototxt, "
                                                ".pbtxt) of the model, or empty"));
    classes_ = declare_parameter("classes", std::string(""),
                                 paramDescriptor("Path or URL of the class names of the model, one name per line, or "
                                                 "empty for the classes of COCO for yolox"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    input_width_ = declare_parameter(
        "input_width", 640, paramDescriptor("Width of the network input, a multiple of 32 for yolox", 1, 4096));
    input_height_ = declare_parameter(
        "input_height", 640, paramDescriptor("Height of the network input, a multiple of 32 for yolox", 1, 4096));
    scale_ = declare_parameter(
        "scale", 1.0 / 255,
        paramDescriptor("Multiplier of the pixel values for the network input, for detection_model", 0.0, 1.0));
    swap_rb_ = declare_parameter(
        "swap_rb", true,
        paramDescriptor("Swap the red and blue channels, true for the models trained in RGB, for detection_model"));
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
