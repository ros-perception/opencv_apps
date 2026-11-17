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

// https://github.com/opencv/opencv_zoo/tree/main/models/image_segmentation_efficientsam
/**
 * Interactive segmentation of the object given by a box and/or points with EfficientSAM-Ti of opencv_zoo (Apache
 * 2.0), as the Segment Anything Model (SAM). The box is given with the mouse or the ~set_roi service as the tracking
 * nodes, and the points by clicking or the ~add_points topic. The prompt is kept until it is changed, and every input
 * image is segmented with it. ~mask is the mask of the object, ~contours its contours and ~output its bounding box.
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/core/version.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/contour.hpp>
#include <opencv_apps/msg/contour_array_stamped.hpp>
#include <opencv_apps/msg/point2_d_array.hpp>
#include <opencv_apps/msg/rect_array_stamped.hpp>
#include <opencv_apps/srv/set_roi.hpp>

#include <algorithm>
#include <cstdio>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#define OPENCV_APPS_CV_VERSION_GE(major, minor)                                                                        \
  (CV_VERSION_MAJOR > (major) || (CV_VERSION_MAJOR == (major) && CV_VERSION_MINOR >= (minor)))

namespace opencv_apps
{
namespace
{
// EfficientSAM-Ti of opencv_zoo (Apache 2.0), the commit is fixed not to change the file
const char* const EFFICIENTSAM_MODEL_URL =
    "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
    "models/image_segmentation_efficientsam/image_segmentation_efficientsam_ti_2025april.onnx";
// the input size of the model, and the number of the points of the prompt (a box is two points)
const int INPUT_SIZE = 1024;
const int MAX_POINTS = 6;
// the labels of the points of the prompt, as efficientSAM.py of opencv_zoo
const float LABEL_PADDING = -1.0f;
const float LABEL_FOREGROUND = 1.0f;
const float LABEL_TOP_LEFT = 2.0f;
const float LABEL_BOTTOM_RIGHT = 3.0f;
}  // namespace

class DNNInteractiveSegmentationNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Publisher mask_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::ContourArrayStamped>::SharedPtr contours_pub_;
  rclcpp::Publisher<opencv_apps::msg::RectArrayStamped>::SharedPtr output_pub_;
  rclcpp::Subscription<opencv_apps::msg::Point2DArray>::SharedPtr add_points_sub_;
  rclcpp::Service<opencv_apps::srv::SetROI>::SharedPtr roi_srv_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  bool window_created_;  // the window has the mouse callback to this node
  std::string window_name_;

  bool use_camera_info_;

  std::string model_;
  std::string backend_;

  std::mutex mutex_;
  cv::dnn::Net net_;
  bool need_model_update_;
  bool read_failed_;  // not to read the model again until the parameters are changed

  // the prompt: the box (empty if none) and the foreground points, in the coordinates of the input image
  cv::Rect box_;
  std::vector<cv::Point2f> points_;

  // selection with mouse, as tracker_nodelet.h: drag for the box, click for a foreground point
  bool selecting_;
  cv::Point selection_start_;
  cv::Rect selection_;
  cv::Size image_size_;

  // mouse events of the window, queued by onMouse() and processed in doWork() of this node, as cv::waitKey() of
  // another node in the same process may call onMouse() of this node
  std::mutex mouse_mutex_;
  std::vector<cv::Vec3i> mouse_events_;  // event, x, y

  static void onMouse(int event, int x, int y, int flags, void* param)
  {
    DNNInteractiveSegmentationNodelet* node = static_cast<DNNInteractiveSegmentationNodelet*>(param);
    std::lock_guard<std::mutex> lock(node->mouse_mutex_);
    node->mouse_events_.push_back(cv::Vec3i(event, x, y));
  }

  // process the queued mouse events, with mutex_ locked
  void processMouseEvents()
  {
    std::vector<cv::Vec3i> events;
    {
      std::lock_guard<std::mutex> lock(mouse_mutex_);
      events.swap(mouse_events_);
    }
    for (const cv::Vec3i& e : events)
    {
      int x = std::min(std::max(e[1], 0), image_size_.width);
      int y = std::min(std::max(e[2], 0), image_size_.height);
      switch (e[0])
      {
        case cv::EVENT_MOUSEMOVE:
          if (selecting_)
            selection_ = cv::Rect(selection_start_, cv::Point(x, y));
          break;
        case cv::EVENT_LBUTTONDOWN:
          selecting_ = true;
          selection_start_ = cv::Point(x, y);
          selection_ = cv::Rect(x, y, 0, 0);
          break;
        case cv::EVENT_LBUTTONUP:
          selecting_ = false;
          selection_ = cv::Rect(selection_start_, cv::Point(x, y));
          // a small drag is a click
          if (selection_.width < 5 && selection_.height < 5)
            addPoints({ cv::Point2f(x, y) });
          else
            setBox(selection_);
          break;
      }
    }
  }

  // a new target of the box, without the points of the previous target, with mutex_ locked
  void setBox(const cv::Rect& box)
  {
    box_ = box;
    points_.clear();
    RCLCPP_INFO(get_logger(), "Segment the box (center-x: %d, center-y: %d, width: %d, height: %d)",
                box.x + box.width / 2, box.y + box.height / 2, box.width, box.height);
  }

  // add the foreground points to the prompt, up to MAX_POINTS with the two corners of the box, with mutex_ locked
  void addPoints(const std::vector<cv::Point2f>& points)
  {
    for (const cv::Point2f& point : points)
    {
      if (static_cast<int>(points_.size()) + (box_.empty() ? 0 : 2) >= MAX_POINTS)
      {
        RCLCPP_WARN(get_logger(), "The prompt has %d points at most, including the two corners of the box", MAX_POINTS);
        break;
      }
      points_.push_back(point);
      RCLCPP_INFO(get_logger(), "Add the foreground point (%.1f, %.1f)", point.x, point.y);
    }
  }

  void setROICb(const std::shared_ptr<opencv_apps::srv::SetROI::Request> req,
                std::shared_ptr<opencv_apps::srv::SetROI::Response> res)
  {
    // x-y of opencv_apps/Rect is the center. The image of the request is not used, as the prompt is used for the
    // next input images.
    cv::Rect roi(cvRound(req->rect.x - req->rect.width / 2), cvRound(req->rect.y - req->rect.height / 2),
                 cvRound(req->rect.width), cvRound(req->rect.height));
    if (roi.empty())
    {
      res->ok = false;
      res->error = "ROI is empty";
      RCLCPP_ERROR_STREAM(get_logger(), res->error);
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    setBox(roi);
    res->ok = true;
  }

  // the points are added to the prompt, and the empty points clear the prompt, as ~add_seed_points of
  // watershed_segmentation
  void addPointsCb(const opencv_apps::msg::Point2DArray& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg.points.empty())
    {
      box_ = cv::Rect();
      points_.clear();
      RCLCPP_INFO(get_logger(), "Clear the prompt");
      return;
    }
    std::vector<cv::Point2f> points;
    for (const opencv_apps::msg::Point2D& point : msg.points)
      points.push_back(cv::Point2f(point.x, point.y));
    addPoints(points);
  }

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

  // (re)create the model with the parameters, or retry when it failed to download, called with mutex_
  bool updateModel()
  {
    if (!need_model_update_ && (!net_.empty() || read_failed_))
      return !read_failed_;
    need_model_update_ = false;
    read_failed_ = false;
    net_ = cv::dnn::Net();
#if !OPENCV_APPS_CV_VERSION_GE(4, 10)
    // cv::dnn of the older OpenCV cannot read the model, e.g. LayerNormalization of 4.6
    RCLCPP_ERROR(get_logger(), "EfficientSAM needs OpenCV 4.10 or later (this is %s)", CV_VERSION);
    read_failed_ = true;
    return false;
#endif
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string model_file = modelFile(model_.empty() ? std::string(EFFICIENTSAM_MODEL_URL) : model_, get_logger());
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

  // segment the object of the prompt on frame, as efficientSAM.py of opencv_zoo, called with mutex_. Returns the
  // mask (0 or 255) of the best mask of the 3 masks of the model by its predicted IoU.
  cv::Mat segment(const cv::Mat& frame, float& iou)
  {
    // RGB of 0..1, resized to 1024x1024 without keeping the aspect ratio
    net_.setInput(cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(INPUT_SIZE, INPUT_SIZE), cv::Scalar(), true,
                                         false),
                  "batched_images");
    // the points in the coordinates of the input of the model, padded with the label -1
    int coords_size[] = { 1, 1, MAX_POINTS, 2 };
    int labels_size[] = { 1, 1, MAX_POINTS };
    cv::Mat coords(4, coords_size, CV_32F, cv::Scalar(0));
    cv::Mat labels(3, labels_size, CV_32F, cv::Scalar(LABEL_PADDING));
    std::vector<std::pair<cv::Point2f, float>> prompt;
    if (!box_.empty())
    {
      prompt.push_back(std::make_pair(cv::Point2f(box_.tl()), LABEL_TOP_LEFT));
      prompt.push_back(std::make_pair(cv::Point2f(box_.br()), LABEL_BOTTOM_RIGHT));
    }
    for (const cv::Point2f& point : points_)
      prompt.push_back(std::make_pair(point, LABEL_FOREGROUND));
    float* coords_data = coords.ptr<float>();
    float* labels_data = labels.ptr<float>();
    for (size_t i = 0; i < prompt.size() && i < static_cast<size_t>(MAX_POINTS); i++)
    {
      coords_data[i * 2] = prompt[i].first.x * INPUT_SIZE / frame.cols;
      coords_data[i * 2 + 1] = prompt[i].first.y * INPUT_SIZE / frame.rows;
      labels_data[i] = prompt[i].second;
    }
    net_.setInput(coords, "batched_point_coords");
    net_.setInput(labels, "batched_point_labels");
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, std::vector<cv::String>{ "output_masks", "iou_predictions" });
    // output_masks: 1 x 1 x masks x height x width (logits), iou_predictions: 1 x 1 x masks
    const cv::Mat& masks = outputs[0];
    const cv::Mat& ious = outputs[1];
    if (masks.dims != 5 || ious.total() != static_cast<size_t>(masks.size[2]))
      CV_Error(cv::Error::StsBadSize, "The outputs of the model are not the masks and their IoU of EfficientSAM");
    int best = 0;
    for (int i = 1; i < masks.size[2]; i++)
    {
      if (ious.ptr<float>()[i] > ious.ptr<float>()[best])
        best = i;
    }
    iou = ious.ptr<float>()[best];
    cv::Mat logits(masks.size[3], masks.size[4], CV_32F,
                   const_cast<float*>(masks.ptr<float>()) + static_cast<size_t>(best) * masks.size[3] * masks.size[4]);
    cv::Mat resized;
    cv::resize(logits, resized, frame.size(), 0, 0, cv::INTER_LINEAR);
    return resized >= 0;
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
      std::lock_guard<std::mutex> lock(mutex_);

      // Do the work
      cv::Mat result_image = frame.clone();
      bool has_prompt = !box_.empty() || !points_.empty();
      cv::Mat mask;
      float iou = 0.0f;
      if (!has_prompt)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Prompt is not defined, please set the box via GUI or ~set_roi service, or the points "
                             "via GUI or ~add_points");
      }
      else if (!updateModel())
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s", model_.c_str());
      }
      else
      {
        mask = segment(frame, iou);
      }

      opencv_apps::msg::ContourArrayStamped contours_msg;
      contours_msg.header = msg->header;
      opencv_apps::msg::RectArrayStamped output_msg;
      output_msg.header = msg->header;
      if (!mask.empty())
      {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const std::vector<cv::Point>& contour : contours)
        {
          opencv_apps::msg::Contour contour_msg;
          for (const cv::Point& p : contour)
          {
            opencv_apps::msg::Point2D point_msg;
            point_msg.x = p.x;
            point_msg.y = p.y;
            contour_msg.points.push_back(point_msg);
          }
          contours_msg.contours.push_back(contour_msg);
        }
        // the bounding box of the mask, x-y is the center as the tracking nodes
        cv::Rect bounding = cv::boundingRect(mask);
        if (!bounding.empty())
        {
          opencv_apps::msg::Rect rect_msg;
          rect_msg.x = bounding.x + bounding.width / 2.0;
          rect_msg.y = bounding.y + bounding.height / 2.0;
          rect_msg.width = bounding.width;
          rect_msg.height = bounding.height;
          output_msg.rects.push_back(rect_msg);
        }

        // the mask is painted in red, with its contours in white
        cv::Mat painted = result_image.clone();
        painted.setTo(cv::Scalar(0, 0, 255), mask);
        cv::addWeighted(result_image, 0.4, painted, 0.6, 0.0, result_image);
        cv::drawContours(result_image, contours, -1, cv::Scalar(255, 255, 255), 2);
        char text[32];
        snprintf(text, sizeof(text), "IoU %.2f", iou);
        cv::putText(result_image, text, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
      }
      // the prompt: the box in blue, and the foreground points in green
      if (!box_.empty())
        cv::rectangle(result_image, box_, cv::Scalar(255, 0, 0), 2);
      for (const cv::Point2f& point : points_)
        cv::circle(result_image, point, 5, cv::Scalar(0, 255, 0), cv::FILLED);

      //-- Show what you got
      if (debug_view_)
      {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        image_size_ = frame.size();
        cv::setMouseCallback(window_name_, onMouse, this);
        window_created_ = true;
        cv::Mat view_image = result_image;
        if (selecting_)
        {
          view_image = result_image.clone();
          cv::rectangle(view_image, selection_, cv::Scalar(0, 0, 255), 2, 1);
        }
        cv::imshow(window_name_, view_image);
        int c = cv::waitKey(1);
        processMouseEvents();
        if (c == 'c' || c == 'C')  // clear the prompt
        {
          box_ = cv::Rect();
          points_.clear();
        }
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, result_image).toImageMsg());
      // publish only when the object is segmented
      if (!mask.empty())
      {
        mask_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg());
        contours_pub_->publish(contours_msg);
        output_pub_->publish(output_msg);
      }
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }
    catch (std::exception& e)  // e.g. cv_bridge::Exception
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s", e.what());
    }
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNInteractiveSegmentationNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNInteractiveSegmentationNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNInteractiveSegmentationNodelet(const rclcpp::NodeOptions& options)
    : Nodelet("dnn_interactive_segmentation", options), selecting_(false)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    // with the namespace, not to share the window with another node in the same process
    window_name_ = std::string("Interactive segmentation (") + get_fully_qualified_name() + ")";
    window_created_ = false;
    need_model_update_ = true;
    read_failed_ = false;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    model_ = declare_parameter(
        "model", std::string(""),
        paramDescriptor("Path or URL of the EfficientSAM model (onnx), empty for EfficientSAM-Ti of opencv_zoo. The "
                        "URL is downloaded to ~/.ros/opencv_apps/models/ at the first time"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNInteractiveSegmentationNodelet::parametersCallback, this, std::placeholders::_1));

    // the prompt is kept, even if the output topics are not subscribed
    add_points_sub_ = create_subscription<opencv_apps::msg::Point2DArray>(
        "~/add_points", 1, std::bind(&DNNInteractiveSegmentationNodelet::addPointsCb, this, std::placeholders::_1));
    // the prompt may be of ~set_roi
    optionalInput("~/add_points");
    roi_srv_ = create_service<opencv_apps::srv::SetROI>("~/set_roi",
                                                        std::bind(&DNNInteractiveSegmentationNodelet::setROICb, this,
                                                                  std::placeholders::_1, std::placeholders::_2));
    img_pub_ = advertiseImage("~/image", 1);
    mask_pub_ = advertiseImage("~/mask", 1);
    contours_pub_ = advertise<opencv_apps::msg::ContourArrayStamped>("~/contours", 1);
    output_pub_ = advertise<opencv_apps::msg::RectArrayStamped>("~/output", 1);

    RCLCPP_INFO(get_logger(), "Hot keys: ");
    RCLCPP_INFO(get_logger(), "\tc - clear the prompt");
    RCLCPP_INFO(get_logger(), "To segment the object, drag a box or click the points on it with mouse, or call "
                              "~set_roi service or publish ~add_points");
    onInitPostProcess();
  }

  virtual ~DNNInteractiveSegmentationNodelet()
  {
    // the window may outlive this node, e.g. unloaded from a component container, then its mouse callback must not
    // call this node
    if (window_created_)
    {
      try
      {
        cv::setMouseCallback(window_name_, nullptr, nullptr);
        cv::destroyWindow(window_name_);
      }
      catch (const cv::Exception&)
      {
      }
    }
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNInteractiveSegmentationNodelet)
