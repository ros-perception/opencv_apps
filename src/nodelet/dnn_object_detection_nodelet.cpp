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
// https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_nanodet
// https://github.com/opencv/opencv/blob/4.x/samples/dnn/yolo_detector.cpp
/**
 * Object detection with YOLOX (onnx), by default YOLOX-S trained on COCO of opencv_zoo, NanoDet-Plus (onnx) or YOLOv8
 * (onnx, the output of YOLOv8, YOLOv9 and YOLO11), or with a model of cv::dnn::DetectionModel, e.g. YOLO (darknet),
 * SSD (caffe, tensorflow), which OpenCV 5 cannot read except onnx
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
// NanoDet-Plus-m-1.5x (416x416) trained on COCO, https://huggingface.co/opencv/opencv_zoo, the commit is fixed
const char* const NANODET_MODEL_URL = "https://huggingface.co/opencv/opencv_zoo/resolve/"
                                      "d4938dfc9d4ec5d098bfa33e98b3f3345a236586/models/object_detection_nanodet/"
                                      "object_detection_nanodet_2022nov.onnx";

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
  cv::dnn::Net net_;                                    // model_type yolox, nanodet and yolov8
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
    if (!need_model_update_ && (model_ptr_ || !net_.empty()))
      return true;
    need_model_update_ = false;
    net_ = cv::dnn::Net();
    model_ptr_.reset();
    class_names_.clear();
    if (model_type_ != "yolox" && model_type_ != "nanodet" && model_type_ != "yolov8" &&
        model_type_ != "detection_model")
    {
      RCLCPP_ERROR(get_logger(), "Unknown model_type %s, use yolox, nanodet, yolov8 or detection_model",
                   model_type_.c_str());
      return false;
    }
    // no default model of yolov8, as the models of Ultralytics are of AGPL-3.0
    if ((model_type_ == "detection_model" || model_type_ == "yolov8") && model_.empty())
    {
      RCLCPP_ERROR(get_logger(), "model is required for model_type %s", model_type_.c_str());
      return false;
    }
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string default_model = model_type_ == "nanodet" ? NANODET_MODEL_URL : YOLOX_MODEL_URL;
    std::string model_file = modelFile(model_.empty() ? default_model : model_, get_logger());
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
    else if (model_type_ != "detection_model")
    {
      class_names_.assign(std::begin(COCO_CLASSES), std::end(COCO_CLASSES));
    }
    if (model_type_ != "detection_model")
    {
      net_ = cv::dnn::readNet(model_file, config_file);
      net_.setPreferableBackend(backend);
      net_.setPreferableTarget(target);
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

  // the frame resized keeping the aspect ratio into the top-left of the input, padded with the value, and the ratio
  cv::Mat letterbox(const cv::Mat& frame, const cv::Size& input_size, const cv::Scalar& padding, double& ratio) const
  {
    ratio = std::min(static_cast<double>(input_size.width) / frame.cols,
                     static_cast<double>(input_size.height) / frame.rows);
    cv::Size resized_size(std::max(1, static_cast<int>(frame.cols * ratio)),
                          std::max(1, static_cast<int>(frame.rows * ratio)));
    cv::Mat input(input_size, CV_8UC3, padding);
    cv::Mat resized = input(cv::Rect(cv::Point(0, 0), resized_size));
    cv::resize(frame, resized, resized_size);
    return input;
  }

  // non maximum suppression of the candidates for each class
  void nmsByClass(const std::vector<int>& candidate_class_ids, const std::vector<float>& candidate_confidences,
                  const std::vector<cv::Rect>& candidate_boxes, std::vector<int>& class_ids,
                  std::vector<float>& confidences, std::vector<cv::Rect>& boxes) const
  {
    std::map<int, std::vector<int> > class_candidates;
    for (size_t i = 0; i < candidate_boxes.size(); i++)
      class_candidates[candidate_class_ids[i]].push_back(static_cast<int>(i));
    for (const std::pair<const int, std::vector<int> >& candidates : class_candidates)
    {
      std::vector<cv::Rect> class_boxes;
      std::vector<float> class_confidences;
      for (int i : candidates.second)
      {
        class_boxes.push_back(candidate_boxes[i]);
        class_confidences.push_back(candidate_confidences[i]);
      }
      std::vector<int> indices;
      cv::dnn::NMSBoxes(class_boxes, class_confidences, static_cast<float>(confidence_threshold_),
                        static_cast<float>(nms_threshold_), indices);
      for (int index : indices)
      {
        int i = candidates.second[index];
        class_ids.push_back(candidate_class_ids[i]);
        confidences.push_back(candidate_confidences[i]);
        boxes.push_back(candidate_boxes[i]);
      }
    }
  }

  // detect the objects by YOLOX, as object_detection_yolox of opencv_zoo, called with mutex_
  void detectYOLOX(const cv::Mat& frame, std::vector<int>& class_ids, std::vector<float>& confidences,
                   std::vector<cv::Rect>& boxes)
  {
    // the image in RGB, resized keeping the aspect ratio into the top-left of the input, padded with 114
    cv::Size input_size(input_width_, input_height_);
    double ratio;
    cv::Mat input = letterbox(frame, input_size, cv::Scalar(114, 114, 114), ratio);
    net_.setInput(cv::dnn::blobFromImage(input, 1.0, cv::Size(), cv::Scalar(), true, false));
    cv::Mat output = net_.forward();  // 1 x N x (4 box + 1 objectness + classes)
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

    nmsByClass(grid_class_ids, grid_confidences, grid_boxes, class_ids, confidences, boxes);
  }

  // detect the objects by NanoDet-Plus, as object_detection_nanodet of opencv_zoo (in BGR, as NanoDet), called with
  // mutex_
  void detectNanoDet(const cv::Mat& frame, std::vector<int>& class_ids, std::vector<float>& confidences,
                     std::vector<cv::Rect>& boxes)
  {
    const int reg_max = 7;  // the distances of the sides of a box are the expectations of the 8 bins
    cv::Size input_size(input_width_, input_height_);
    double ratio;
    cv::Mat input;
    letterbox(frame, input_size, cv::Scalar(0, 0, 0), ratio).convertTo(input, CV_32FC3);
    cv::subtract(input, cv::Scalar(103.53, 116.28, 123.675), input);
    cv::divide(input, cv::Scalar(57.375, 57.12, 58.395), input);
    net_.setInput(cv::dnn::blobFromImage(input));
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    // the outputs are the scores of the classes and the distributions of the distances of each stride, of N rows of
    // the grid of the stride; the order of the outputs depends on the version of OpenCV, so they are paired by N
    std::map<int, std::vector<cv::Mat> > grids;
    for (const cv::Mat& output : outputs)
      if (output.dims == 3)
        grids[output.size[1]].push_back(
            cv::Mat(output.size[1], output.size[2], CV_32F, const_cast<float*>(output.ptr<float>())));
    std::vector<int> candidate_class_ids;
    std::vector<float> candidate_confidences;
    std::vector<cv::Rect> candidate_boxes;
    for (const std::pair<const int, std::vector<cv::Mat> >& grid : grids)
    {
      // the distances have 4 x (reg_max + 1) columns, and the scores are in [0, 1] (sigmoid), also when the number of
      // the classes is the same as the columns of the distances
      std::vector<bool> in_unit(grid.second.size());
      for (size_t i = 0; i < grid.second.size(); i++)
      {
        double min_value, max_value;
        cv::minMaxLoc(grid.second[i], &min_value, &max_value);
        in_unit[i] = min_value >= 0.0 && max_value <= 1.0;
      }
      int score_index = -1;
      if (grid.second.size() == 2)
      {
        bool regression0 = grid.second[0].cols == 4 * (reg_max + 1),
             regression1 = grid.second[1].cols == 4 * (reg_max + 1);
        if (regression0 != regression1)
          score_index = regression0 ? 1 : 0;
        else if (regression0 && in_unit[0] != in_unit[1])
          score_index = in_unit[0] ? 0 : 1;
      }
      // the stride of the grid of the supported strides, of the feature map of floor or ceil of the input size
      int stride = 0, grid_width = 0;
      for (int s : { 8, 16, 32, 64 })
      {
        for (bool ceil : { false, true })
        {
          int width = ceil ? (input_size.width + s - 1) / s : input_size.width / s;
          int height = ceil ? (input_size.height + s - 1) / s : input_size.height / s;
          if (stride == 0 && width * height == grid.first)
          {
            stride = s;
            grid_width = width;
          }
        }
      }
      if (score_index < 0 || stride == 0)
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                              "The outputs of the model do not match NanoDet of the input size %dx%d (the scores and "
                              "the distances of %d rows)",
                              input_size.width, input_size.height, grid.first);
        return;
      }
      std::pair<int, cv::Mat> score(grid.first, grid.second[score_index]);
      const cv::Mat& distance = grid.second[1 - score_index];
      for (int row = 0; row < score.first; row++)
      {
        cv::Point class_id;
        double confidence;
        cv::minMaxLoc(score.second.row(row), nullptr, &confidence, nullptr, &class_id);
        if (confidence < confidence_threshold_)
          continue;
        // the expectation of the softmax of the bins of each side (left, top, right, bottom)
        double sides[4];
        const float* d = distance.ptr<float>(row);
        for (int side = 0; side < 4; side++)
        {
          const float* bins = d + side * (reg_max + 1);
          float max_bin = *std::max_element(bins, bins + reg_max + 1);
          double sum = 0.0, expectation = 0.0;
          for (int bin = 0; bin <= reg_max; bin++)
          {
            double e = std::exp(bins[bin] - max_bin);
            sum += e;
            expectation += bin * e;
          }
          sides[side] = expectation / sum * stride;
        }
        double cx = (row % grid_width) * stride + 0.5 * (stride - 1);
        double cy = (row / grid_width) * stride + 0.5 * (stride - 1);
        double x1 = std::max(0.0, cx - sides[0]), y1 = std::max(0.0, cy - sides[1]);
        double x2 = std::min<double>(input_size.width, cx + sides[2]);
        double y2 = std::min<double>(input_size.height, cy + sides[3]);
        candidate_class_ids.push_back(class_id.x);
        candidate_confidences.push_back(static_cast<float>(confidence));
        candidate_boxes.push_back(
            cv::Rect(cvRound(x1 / ratio), cvRound(y1 / ratio), cvRound((x2 - x1) / ratio), cvRound((y2 - y1) / ratio)));
      }
    }
    nmsByClass(candidate_class_ids, candidate_confidences, candidate_boxes, class_ids, confidences, boxes);
  }

  // detect the objects by YOLOv8 (and YOLOv9, YOLO11 of the same output), as yolo_detector.cpp of OpenCV, called
  // with mutex_
  void detectYOLOv8(const cv::Mat& frame, std::vector<int>& class_ids, std::vector<float>& confidences,
                    std::vector<cv::Rect>& boxes)
  {
    // the image in RGB in [0, 1], resized keeping the aspect ratio into the top-left of the input, padded with 114
    cv::Size input_size(input_width_, input_height_);
    double ratio;
    cv::Mat input = letterbox(frame, input_size, cv::Scalar(114, 114, 114), ratio);
    net_.setInput(cv::dnn::blobFromImage(input, 1.0 / 255, cv::Size(), cv::Scalar(), true, false));
    cv::Mat output = net_.forward();  // 1 x (4 box + classes) x N
    if (output.dims != 3 || output.size[1] <= 4)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "The output of the model is not of YOLOv8");
      return;
    }
    cv::Mat predictions = cv::Mat(output.size[1], output.size[2], CV_32F, output.ptr<float>()).t();  // N x (4 + C)
    std::vector<int> candidate_class_ids;
    std::vector<float> candidate_confidences;
    std::vector<cv::Rect> candidate_boxes;
    for (int row = 0; row < predictions.rows; row++)
    {
      const float* p = predictions.ptr<float>(row);
      cv::Point class_id;
      double confidence;
      cv::minMaxLoc(cv::Mat(1, predictions.cols - 4, CV_32F, const_cast<float*>(p + 4)), nullptr, &confidence, nullptr,
                    &class_id);
      if (confidence < confidence_threshold_)
        continue;
      // the center and the size of the box in the input
      double width = p[2] / ratio, height = p[3] / ratio;
      candidate_class_ids.push_back(class_id.x);
      candidate_confidences.push_back(static_cast<float>(confidence));
      candidate_boxes.push_back(cv::Rect(cvRound(p[0] / ratio - width / 2), cvRound(p[1] / ratio - height / 2),
                                         cvRound(width), cvRound(height)));
    }
    nmsByClass(candidate_class_ids, candidate_confidences, candidate_boxes, class_ids, confidences, boxes);
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
        else if (model_type_ == "nanodet")
          detectNanoDet(frame, class_ids, confidences, boxes);
        else if (model_type_ == "yolov8")
          detectYOLOv8(frame, class_ids, confidences, boxes);
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

          // a bright color of each class, the hue of the class id
          cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar((class_ids[i] * 47 + 60) % 180, 255, 255)), bgr;
          cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
          cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
          cv::Scalar color(c[0], c[1], c[2]);
          // the line and the text relative to the image, to be read on large images
          double font_scale = std::max(0.6, frame.cols / 700.0);
          int thickness = std::max(2, cvRound(frame.cols / 240.0));
          cv::rectangle(frame, box, color, thickness);
          std::string text = cv::format("%s %.2f", detection_msg.label.c_str(), confidences[i]);
          int baseline;
          int text_thickness = std::max(1, cvRound(font_scale * 1.5));
          cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, text_thickness, &baseline);
          int margin = cvRound(4 * font_scale);
          cv::Point origin(box.x + margin, std::max(box.y - margin, text_size.height + margin));
          cv::rectangle(frame,
                        cv::Rect(origin.x - margin, origin.y - text_size.height - margin, text_size.width + 2 * margin,
                                 text_size.height + baseline + margin),
                        color, cv::FILLED);
          cv::putText(frame, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 0, 0), text_thickness);
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
        paramDescriptor("yolox for the YOLOX models (onnx), nanodet for NanoDet-Plus (onnx, input 416x416), yolov8 "
                        "for the models of the output of YOLOv8, YOLOv9 and YOLO11 (onnx, OpenCV 4.9 or later), or "
                        "detection_model for the models of cv::dnn::DetectionModel, e.g. YOLO (darknet), SSD (caffe, "
                        "tensorflow), with config, input_width, input_height, scale and swap_rb"));
    model_ = declare_parameter(
        "model", std::string(""),
        paramDescriptor("Path or URL of the model (e.g. .onnx, .weights, .caffemodel, .pb), empty for YOLOX-S "
                        "(yolox) or NanoDet-Plus (nanodet) trained on COCO of opencv_zoo. The URL is downloaded to "
                        "~/.ros/opencv_apps/models/ at the first time"));
    config_ = declare_parameter("config", std::string(""),
                                paramDescriptor("Path or URL of the network configuration (e.g. .cfg, .prototxt, "
                                                ".pbtxt) of the model, or empty"));
    classes_ = declare_parameter("classes", std::string(""),
                                 paramDescriptor("Path or URL of the class names of the model, one name per line, or "
                                                 "empty for the classes of COCO for yolox, nanodet and yolov8"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    input_width_ = declare_parameter(
        "input_width", 640,
        paramDescriptor("Width of the network input, a multiple of 32 for yolox and yolov8, 416 for nanodet", 1, 4096));
    input_height_ = declare_parameter(
        "input_height", 640,
        paramDescriptor("Height of the network input, a multiple of 32 for yolox and yolov8, 416 for nanodet", 1, 4096));
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
