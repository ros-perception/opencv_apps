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

// https://github.com/opencv/opencv/blob/4.x/samples/dnn/person_reid.py
// https://github.com/opencv/opencv_zoo/tree/main/models/person_reid_youtureid
/**
 * Person re-identification of the people detected by another node (e.g. ~/detections of dnn_object_detection,
 * synchronized with the image) with the YouTu ReID baseline of opencv_zoo (Apache 2.0): the feature of each person is
 * compared with the gallery of the features of the people seen before by the cosine similarity, and the person gets
 * the id of the most similar one over match_threshold, or a new id. Each person of the gallery is matched with one
 * person of an image, the most similar pairs first, and its feature is updated by the moving average. The detections
 * are published with the id of the person in their id.
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/version.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>

#include <opencv_apps/msg/detection.hpp>
#include <opencv_apps/msg/detection_array_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

namespace opencv_apps
{
namespace
{
// YouTu ReID baseline of opencv_zoo, the commit is fixed not to change the file
const char* const YOUTU_REID_MODEL_URL = "https://github.com/opencv/opencv_zoo/raw/"
                                         "48b10fa505aeb51fbd127fc60a5c660040e823b7/models/person_reid_youtureid/"
                                         "person_reid_youtu_2021nov.onnx";
}  // namespace

class DNNPersonReIDNodelet : public opencv_apps::Nodelet
{
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, opencv_apps::msg::DetectionArrayStamped>
      SyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, opencv_apps::msg::DetectionArrayStamped>
      ApproximateSyncPolicy;

  image_transport::Publisher img_pub_;
  image_transport::SubscriberFilter img_sub_;
  message_filters::Subscriber<opencv_apps::msg::DetectionArrayStamped> detections_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
  rclcpp::Publisher<opencv_apps::msg::DetectionArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  bool use_async_;

  std::string model_;
  std::string backend_;
  std::string label_;
  double match_threshold_;
  double update_rate_;

  std::mutex mutex_;
  cv::dnn::Net net_;
  bool need_model_update_;
  // the gallery: the id and the feature (L2 normalized) of each person seen before
  struct Person
  {
    int id;
    cv::Mat feature;
  };
  std::vector<Person> gallery_;
  int next_id_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "label")
        label_ = parameter.as_string();
      else if (parameter.get_name() == "match_threshold")
        match_threshold_ = parameter.as_double();
      else if (parameter.get_name() == "update_rate")
        update_rate_ = parameter.as_double();
      else if (parameter.get_name() == "model" || parameter.get_name() == "backend")
      {
        if (parameter.get_name() == "model")
          model_ = parameter.as_string();
        else
          backend_ = parameter.as_string();
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
    if (!need_model_update_ && !net_.empty())
      return true;
    need_model_update_ = false;
    net_ = cv::dnn::Net();
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string model_file = modelFile(model_.empty() ? std::string(YOUTU_REID_MODEL_URL) : model_, get_logger());
    if (model_file.empty())
      return false;
    net_ = cv::dnn::readNet(model_file);
    net_.setPreferableBackend(backend);
    net_.setPreferableTarget(target);
    return true;
  }

  // the L2 normalized features of the people in the rects, called with mutex_
  std::vector<cv::Mat> features(const cv::Mat& frame, const std::vector<cv::Rect>& rects)
  {
    // the input of the model: 128x256 RGB, normalized by the mean and the std of ImageNet
    const cv::Scalar mean(0.485, 0.456, 0.406), stddev(0.229, 0.224, 0.225);
    std::vector<cv::Mat> crops;
    for (const cv::Rect& rect : rects)
    {
      cv::Mat crop;
      cv::resize(frame(rect), crop, cv::Size(128, 256));
      cv::cvtColor(crop, crop, cv::COLOR_BGR2RGB);
      crop.convertTo(crop, CV_32FC3, 1.0 / 255.0);
      cv::subtract(crop, mean, crop);
      cv::divide(crop, stddev, crop);
      crops.push_back(crop);
    }
    std::vector<cv::Mat> features;
    if (crops.empty())
      return features;
    net_.setInput(cv::dnn::blobFromImages(crops));
    cv::Mat output = net_.forward();  // N x 768 x 1 x 1
    // the rows of the features, on the data of output, which owns it
    int rows = static_cast<int>(rects.size());
    cv::Mat output_rows(rows, static_cast<int>(output.total()) / rows, CV_32F, output.ptr<float>());
    for (int i = 0; i < output_rows.rows; i++)
    {
      cv::Mat feature;
      cv::normalize(output_rows.row(i), feature);
      features.push_back(feature);
    }
    return features;
  }

  // the index of the person in the gallery of each feature, a new person if no one is similar enough, and its
  // similarity (NaN for a new person); the most similar pairs are matched first, so that each person of the gallery
  // has one feature of the image, called with mutex_
  std::vector<std::pair<int, double> > match(const std::vector<cv::Mat>& features)
  {
    std::vector<std::tuple<double, int, int> > pairs;  // similarity, index of the feature and of the gallery
    for (size_t i = 0; i < features.size(); i++)
      for (size_t j = 0; j < gallery_.size(); j++)
        pairs.emplace_back(features[i].dot(gallery_[j].feature), static_cast<int>(i), static_cast<int>(j));
    std::sort(pairs.begin(), pairs.end(),
              [](const std::tuple<double, int, int>& a, const std::tuple<double, int, int>& b) {
                return std::get<0>(a) > std::get<0>(b);
              });
    std::vector<std::pair<int, double> > matched(features.size(), std::make_pair(-1, std::nan("")));
    std::vector<bool> used(gallery_.size(), false);
    for (const std::tuple<double, int, int>& pair : pairs)
    {
      double similarity;
      int i, j;
      std::tie(similarity, i, j) = pair;
      if (similarity < match_threshold_)
        break;
      if (matched[i].first >= 0 || used[j])
        continue;
      matched[i] = std::make_pair(j, similarity);
      used[j] = true;
    }
    // update the features of the matched people by the moving average, and add the new people
    for (size_t i = 0; i < features.size(); i++)
    {
      if (matched[i].first >= 0)
      {
        Person& person = gallery_[matched[i].first];
        cv::normalize((1.0 - update_rate_) * person.feature + update_rate_ * features[i], person.feature);
      }
      else
      {
        matched[i].first = static_cast<int>(gallery_.size());
        gallery_.push_back({ next_id_++, features[i].clone() });
      }
    }
    return matched;
  }

  static cv::Scalar idColor(int id)
  {
    return cv::Scalar((id * 67) % 256, (id * 131 + 100) % 256, (id * 197 + 50) % 256);
  }

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                const opencv_apps::msg::DetectionArrayStamped::ConstSharedPtr& detections_msg)
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
      opencv_apps::msg::DetectionArrayStamped reid_msg;
      reid_msg.header = detections_msg->header;

      std::lock_guard<std::mutex> lock(mutex_);
      if (!updateModel())
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Model is not loaded: %s", model_.c_str());
        return;
      }

      // the detections of the label (all of them if empty) in the image, x-y of the rect is the center
      std::vector<cv::Rect> rects;
      for (const opencv_apps::msg::Detection& detection : detections_msg->detections)
      {
        if (!label_.empty() && detection.label != label_)
          continue;
        cv::Rect rect(cvRound(detection.rect.x - detection.rect.width / 2),
                      cvRound(detection.rect.y - detection.rect.height / 2), cvRound(detection.rect.width),
                      cvRound(detection.rect.height));
        rect &= cv::Rect(0, 0, frame.cols, frame.rows);
        if (rect.empty())
          continue;
        rects.push_back(rect);
        reid_msg.detections.push_back(detection);
      }

      // Do the work, the id of the person is in the id of the detection
      std::vector<std::pair<int, double> > matched = match(features(frame, rects));
      for (size_t i = 0; i < rects.size(); i++)
      {
        int id = gallery_[matched[i].first].id;
        opencv_apps::msg::Detection& detection = reid_msg.detections[i];
        detection.id = id;
        cv::Scalar color = idColor(id);
        cv::rectangle(frame, rects[i], color, 2);
        // the similarity to the person in the gallery, or "new"
        std::string text = !std::isnan(matched[i].second) ?
                               cv::format("%s_%d %.2f", detection.label.c_str(), id, matched[i].second) :
                               cv::format("%s_%d new", detection.label.c_str(), id);
        int baseline = 0;
        cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::Point origin(rects[i].x, std::max(rects[i].y, size.height + 4));
        cv::rectangle(frame, cv::Rect(origin.x, origin.y - size.height - 4, size.width + 4, size.height + 4), color,
                      cv::FILLED);
        cv::putText(frame, text, cv::Point(origin.x + 2, origin.y - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Person ReID", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      img_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg());
      msg_pub_->publish(reid_msg);
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
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "subscribe");
    // image_transport takes rclcpp::QoS since lyrical, message_filters since kilted
#if __has_include(<image_transport/node_interfaces.hpp>)
    img_sub_.subscribe(*this, "image", "raw", subscriberQos(queue_size_));
#else
    img_sub_.subscribe(this, "image", "raw", subscriberQos(queue_size_).get_rmw_qos_profile());
#endif
#if RCLCPP_VERSION_GTE(29, 0, 0)
    detections_sub_.subscribe(this, "input", subscriberQos(queue_size_));
#else
    detections_sub_.subscribe(this, "input", subscriberQos(queue_size_).get_rmw_qos_profile());
#endif
    if (use_async_)
    {
      async_ =
          std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(ApproximateSyncPolicy(queue_size_));
      async_->connectInput(img_sub_, detections_sub_);
      async_->registerCallback(
          std::bind(&DNNPersonReIDNodelet::callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(queue_size_));
      sync_->connectInput(img_sub_, detections_sub_);
      sync_->registerCallback(
          std::bind(&DNNPersonReIDNodelet::callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "unsubscribe");
    img_sub_.unsubscribe();
    detections_sub_.unsubscribe();
  }

public:
  ~DNNPersonReIDNodelet() override
  {
    sync_.reset();
    async_.reset();
  }

  explicit DNNPersonReIDNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_person_reid", options)
  {
    queue_size_ = declare_parameter("queue_size", 10);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    need_model_update_ = true;
    next_id_ = 0;

    use_async_ = declare_parameter(
        "approximate_sync", false,
        paramDescriptor("Synchronize the image and the detections by the approximate time of the stamps, instead "
                        "of the exact time"));
    model_ = declare_parameter("model", std::string(""),
                               paramDescriptor("Path or URL of the person ReID model (onnx, 128x256 RGB input), empty "
                                               "for the YouTu ReID model of opencv_zoo. The URL is downloaded to "
                                               "~/.ros/opencv_apps/models/ at the first time"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    label_ = declare_parameter("label", std::string("person"),
                               paramDescriptor("Label of the detections to re-identify, empty for all of them"));
    match_threshold_ = declare_parameter(
        "match_threshold", 0.5,
        paramDescriptor("Threshold of the cosine similarity to the person seen before to have the same id, a new id "
                        "otherwise",
                        -1.0, 1.0));
    update_rate_ = declare_parameter(
        "update_rate", 0.1,
        paramDescriptor("Rate of the feature of the new image in the moving average of the feature of the person", 0.0,
                        1.0));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNPersonReIDNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::DetectionArrayStamped>("~/detections", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNPersonReIDNodelet)
