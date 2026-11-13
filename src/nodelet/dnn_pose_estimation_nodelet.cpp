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

// https://github.com/opencv/opencv_zoo/tree/main/models/pose_estimation_mediapipe
// https://github.com/opencv/opencv_zoo/tree/main/models/person_detection_mediapipe
/**
 * Human pose estimation with MediaPipe BlazePose of opencv_zoo (Apache 2.0): the persons are detected by the person
 * detection model, then the 33 landmarks of each person are estimated by the pose estimation model, on the image
 * rotated so that the person stands upright. The models need OpenCV 4.9 or later (OpenCV 4.6 cannot read them).
 *
 * ~detections has a Detection for each person: label "person", id -1, score the confidence of the pose, rect the
 * bounding box of the landmarks, and corners the 33 landmarks in the image coordinates, in the order of MediaPipe
 * (0 nose, 1-3 left eye inner / eye / outer, 4-6 right eye inner / eye / outer, 7 left ear, 8 right ear, 9-10 mouth
 * left / right, 11 left shoulder, 12 right shoulder, 13 left elbow, 14 right elbow, 15 left wrist, 16 right wrist,
 * 17-22 pinky, index and thumb of the left / right hands, 23 left hip, 24 right hip, 25 left knee, 26 right knee,
 * 27 left ankle, 28 right ankle, 29 left heel, 30 right heel, 31 left foot index, 32 right foot index).
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
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace opencv_apps
{
namespace
{
// the models of https://github.com/opencv/opencv_zoo, the commit is fixed not to change the files
const char* const PERSON_DETECTION_MODEL_URL =
    "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
    "models/person_detection_mediapipe/person_detection_mediapipe_2023mar.onnx";
const char* const POSE_ESTIMATION_MODEL_URL =
    "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
    "models/pose_estimation_mediapipe/pose_estimation_mediapipe_2023mar.onnx";

const int PERSON_DETECTION_INPUT_SIZE = 224;
const int POSE_ESTIMATION_INPUT_SIZE = 256;
// 33 landmarks and 6 auxiliary points
const int NUM_LANDMARKS = 33;
const int NUM_POSE_POINTS = 39;
// the box of the landmarks is enlarged by this factor, as opencv_zoo
const double PERSON_BOX_ENLARGE_FACTOR = 1.25;
// only the landmarks of this presence are drawn, as opencv_zoo
const double LANDMARK_PRESENCE_THRESHOLD = 0.8;

// the lines between the landmarks to draw the skeleton
const int SKELETON[][2] = { { 0, 1 },   { 1, 2 },   { 2, 3 },   { 3, 7 },   { 0, 4 },   { 4, 5 },   { 5, 6 },
                            { 6, 8 },   { 9, 10 },  { 12, 14 }, { 14, 16 }, { 16, 22 }, { 16, 18 }, { 16, 20 },
                            { 18, 20 }, { 11, 13 }, { 13, 15 }, { 15, 21 }, { 15, 19 }, { 15, 17 }, { 17, 19 },
                            { 11, 12 }, { 11, 23 }, { 23, 24 }, { 24, 12 }, { 24, 26 }, { 26, 28 }, { 28, 30 },
                            { 28, 32 }, { 30, 32 }, { 23, 25 }, { 25, 27 }, { 27, 31 }, { 27, 29 }, { 29, 31 } };

// the anchors (normalized centers) of the person detection model, as SsdAnchorsCalculator of MediaPipe for the
// 224x224 input: the layers of strides 8, 16, 32, 32 and 32 have 2 anchors at each cell, and the layers of the same
// stride share the cells, so 2, 2 and 6 anchors for the strides 8, 16 and 32
std::vector<cv::Point2f> personDetectionAnchors()
{
  std::vector<cv::Point2f> anchors;
  const int strides[][2] = { { 8, 2 }, { 16, 2 }, { 32, 6 } };
  for (const auto& stride : strides)
  {
    int cells = PERSON_DETECTION_INPUT_SIZE / stride[0];
    for (int y = 0; y < cells; y++)
      for (int x = 0; x < cells; x++)
        for (int i = 0; i < stride[1]; i++)
          anchors.push_back(cv::Point2f((x + 0.5f) / cells, (y + 0.5f) / cells));
  }
  return anchors;
}

// the output of the net of the size, as the order of the outputs depends on the version of OpenCV
cv::Mat outputOfSize(const std::vector<cv::Mat>& outputs, size_t size)
{
  for (const cv::Mat& output : outputs)
    if (output.total() == size)
      return output;
  return cv::Mat();
}
}  // namespace

class DNNPoseEstimationNodelet : public opencv_apps::Nodelet
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

  std::string person_detection_model_;
  std::string pose_estimation_model_;
  std::string backend_;
  double confidence_threshold_;
  double nms_threshold_;

  std::mutex mutex_;
  cv::dnn::Net person_detection_;
  cv::dnn::Net pose_estimation_;
  std::vector<cv::Point2f> anchors_;
  bool need_model_update_;
  bool read_failed_;  // the models could not be read, not retried until the parameters change

  // a person detected by the person detection model
  struct Person
  {
    float score;
    cv::Point2f mid_hip;    // the center of the full body
    cv::Point2f full_body;  // the point on the circle of the full body
  };

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
        if (parameter.get_name() == "person_detection_model")
          person_detection_model_ = parameter.as_string();
        else if (parameter.get_name() == "pose_estimation_model")
          pose_estimation_model_ = parameter.as_string();
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

  // (re)create the models with the parameters, or retry when they failed to load, called with mutex_
  bool updateModel()
  {
    if (!need_model_update_ && ((!person_detection_.empty() && !pose_estimation_.empty()) || read_failed_))
      return !read_failed_;
    need_model_update_ = false;
    read_failed_ = false;
    person_detection_ = cv::dnn::Net();
    pose_estimation_ = cv::dnn::Net();
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string person_detection_file =
        modelFile(person_detection_model_.empty() ? std::string(PERSON_DETECTION_MODEL_URL) : person_detection_model_,
                  get_logger());
    std::string pose_estimation_file = modelFile(
        pose_estimation_model_.empty() ? std::string(POSE_ESTIMATION_MODEL_URL) : pose_estimation_model_, get_logger());
    if (person_detection_file.empty() || pose_estimation_file.empty())
      return false;
    try
    {
      person_detection_ = cv::dnn::readNet(person_detection_file);
      pose_estimation_ = cv::dnn::readNet(pose_estimation_file);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to read the models, which need OpenCV 4.9 or later (this is %s): %s",
                   CV_VERSION, e.what());
      person_detection_ = cv::dnn::Net();
      pose_estimation_ = cv::dnn::Net();
      read_failed_ = true;
      return false;
    }
    for (cv::dnn::Net* net : { &person_detection_, &pose_estimation_ })
    {
      net->setPreferableBackend(backend);
      net->setPreferableTarget(target);
    }
    return true;
  }

  // detect the persons, as person_detection_mediapipe of opencv_zoo, called with mutex_
  std::vector<Person> detectPersons(const cv::Mat& frame)
  {
    // the image in RGB, normalized to -1..1 and resized keeping the aspect ratio into the center of the input,
    // padded with 0 after the normalization, as opencv_zoo
    const int input_size = PERSON_DETECTION_INPUT_SIZE;
    double ratio = std::min(static_cast<double>(input_size) / frame.cols, static_cast<double>(input_size) / frame.rows);
    cv::Size resized_size(std::max(1, static_cast<int>(frame.cols * ratio)),
                          std::max(1, static_cast<int>(frame.rows * ratio)));
    cv::Point pad((input_size - resized_size.width) / 2, (input_size - resized_size.height) / 2);
    cv::Mat resized;
    cv::resize(frame, resized, resized_size);
    cv::Mat input(input_size, input_size, CV_32FC3, cv::Scalar::all(0));
    cv::Mat roi = input(cv::Rect(pad, resized_size));
    resized.convertTo(roi, CV_32F, 1.0 / 127.5, -1.0);
    person_detection_.setInput(cv::dnn::blobFromImage(input, 1.0, cv::Size(), cv::Scalar(), true, false));
    std::vector<cv::Mat> outputs;
    person_detection_.forward(outputs, person_detection_.getUnconnectedOutLayersNames());
    // 1 x anchors x (4 box + 4 x 2 keypoints), and 1 x anchors x 1 score
    cv::Mat deltas = outputOfSize(outputs, anchors_.size() * 12);
    cv::Mat scores = outputOfSize(outputs, anchors_.size());
    if (deltas.empty() || scores.empty())
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                            "The outputs of the model are not of MediaPipe person detection");
      return std::vector<Person>();
    }
    deltas = deltas.reshape(1, static_cast<int>(anchors_.size()));
    scores = scores.reshape(1, static_cast<int>(anchors_.size()));

    // the normalized coordinates of the input to the image
    double scale = input_size / ratio;
    cv::Point2f offset(static_cast<float>(pad.x / ratio), static_cast<float>(pad.y / ratio));
    auto point = [&](const float* delta, const cv::Point2f& anchor) {
      return cv::Point2f(static_cast<float>((delta[0] / input_size + anchor.x) * scale - offset.x),
                         static_cast<float>((delta[1] / input_size + anchor.y) * scale - offset.y));
    };
    std::vector<cv::Rect2d> boxes;
    std::vector<float> confidences;
    std::vector<Person> candidates;
    for (int i = 0; i < deltas.rows; i++)
    {
      float score = 1.0f / (1.0f + std::exp(-std::max(-100.0f, std::min(100.0f, scores.at<float>(i)))));
      if (score < confidence_threshold_)
        continue;
      const float* delta = deltas.ptr<float>(i);
      cv::Point2f center = point(delta, anchors_[i]);
      cv::Point2f size(static_cast<float>(delta[2] / input_size * scale),
                       static_cast<float>(delta[3] / input_size * scale));
      boxes.push_back(cv::Rect2d(center.x - size.x / 2, center.y - size.y / 2, size.x, size.y));
      confidences.push_back(score);
      // the keypoints: mid hip, full body, mid shoulder and upper body
      candidates.push_back(Person{ score, point(delta + 4, anchors_[i]), point(delta + 6, anchors_[i]) });
    }
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, static_cast<float>(confidence_threshold_), static_cast<float>(nms_threshold_),
                      indices);
    std::vector<Person> persons;
    for (int i : indices)
      persons.push_back(candidates[i]);
    return persons;
  }

  // estimate the pose of the person, as pose_estimation_mediapipe of opencv_zoo, called with mutex_
  // landmarks: NUM_POSE_POINTS x (x, y, z, visibility, presence) in the image coordinates
  bool estimatePose(const cv::Mat& frame, const Person& person, cv::Mat& landmarks, float& confidence)
  {
    // the square around the full body, cropped from the image and padded
    float radius = static_cast<float>(cv::norm(person.mid_hip - person.full_body));
    cv::Rect full_box(cvRound(person.mid_hip.x - radius), cvRound(person.mid_hip.y - radius), cvRound(radius * 2),
                      cvRound(radius * 2));
    if (full_box.width <= 0)
      return false;
    cv::Rect crop_box = full_box & cv::Rect(0, 0, frame.cols, frame.rows);
    if (crop_box.empty())
      return false;
    cv::Mat image;
    cv::copyMakeBorder(frame(crop_box), image, crop_box.y - full_box.y, full_box.br().y - crop_box.br().y,
                       crop_box.x - full_box.x, full_box.br().x - crop_box.x - crop_box.width, cv::BORDER_CONSTANT,
                       cv::Scalar::all(0));
    cv::Point2f origin(static_cast<float>(full_box.x), static_cast<float>(full_box.y));

    // rotate the image so that the person stands upright, around the mid hip
    cv::Point2f mid_hip = person.mid_hip - origin;
    cv::Point2f full_body = person.full_body - origin;
    double radians = CV_PI / 2 - std::atan2(-(full_body.y - mid_hip.y), full_body.x - mid_hip.x);
    radians -= 2 * CV_PI * std::floor((radians + CV_PI) / (2 * CV_PI));
    double angle = radians * 180 / CV_PI;
    cv::Mat rotation = cv::getRotationMatrix2D(mid_hip, angle, 1.0);
    cv::Mat rotated;
    cv::warpAffine(image, rotated, rotation, image.size());

    // the input in NHWC, RGB, in 0..1
    const int input_size = POSE_ESTIMATION_INPUT_SIZE;
    cv::Mat input;
    cv::resize(rotated, input, cv::Size(input_size, input_size));
    cv::cvtColor(input, input, cv::COLOR_BGR2RGB);
    input.convertTo(input, CV_32F, 1.0 / 255);
    const int shape[] = { 1, input_size, input_size, 3 };
    pose_estimation_.setInput(cv::Mat(4, shape, CV_32F, input.ptr<float>()));
    std::vector<cv::Mat> outputs;
    pose_estimation_.forward(outputs, pose_estimation_.getUnconnectedOutLayersNames());
    // NUM_POSE_POINTS x 5 landmarks and 1 confidence, (and the mask, the heatmap and the world landmarks)
    cv::Mat pose = outputOfSize(outputs, NUM_POSE_POINTS * 5);
    cv::Mat pose_confidence = outputOfSize(outputs, 1);
    if (pose.empty() || pose_confidence.empty())
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                            "The outputs of the model are not of MediaPipe pose estimation");
      return false;
    }
    confidence = pose_confidence.at<float>(0);
    if (confidence < confidence_threshold_)
      return false;

    // the landmarks in the input to the image: scale to the rotated image, rotate back around its center
    landmarks = pose.reshape(1, NUM_POSE_POINTS).clone();
    double scale = static_cast<double>(image.cols) / input_size;
    cv::Mat inverse;
    cv::invertAffineTransform(rotation, inverse);
    cv::Point2d center(image.cols / 2.0, image.rows / 2.0);
    cv::Point2d original_center(
        inverse.at<double>(0, 0) * center.x + inverse.at<double>(0, 1) * center.y + inverse.at<double>(0, 2),
        inverse.at<double>(1, 0) * center.x + inverse.at<double>(1, 1) * center.y + inverse.at<double>(1, 2));
    double cos_a = std::cos(radians), sin_a = std::sin(radians);
    for (int i = 0; i < landmarks.rows; i++)
    {
      float* landmark = landmarks.ptr<float>(i);
      double x = (landmark[0] - input_size / 2.0) * scale;
      double y = (landmark[1] - input_size / 2.0) * scale;
      landmark[0] = static_cast<float>(x * cos_a - y * sin_a + original_center.x + origin.x);
      landmark[1] = static_cast<float>(x * sin_a + y * cos_a + original_center.y + origin.y);
      landmark[2] = static_cast<float>(landmark[2] * scale);
      // visibility and presence
      landmark[3] = 1.0f / (1.0f + std::exp(-landmark[3]));
      landmark[4] = 1.0f / (1.0f + std::exp(-landmark[4]));
    }
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

      // Messages
      opencv_apps::msg::DetectionArrayStamped detections_msg;
      detections_msg.header = msg->header;

      // Do the work
      std::vector<cv::Mat> poses;
      std::vector<float> confidences;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Models are not loaded");
          return;
        }
        for (const Person& person : detectPersons(frame))
        {
          cv::Mat landmarks;
          float confidence;
          if (estimatePose(frame, person, landmarks, confidence))
          {
            poses.push_back(landmarks);
            confidences.push_back(confidence);
          }
        }
      }

      for (size_t i = 0; i < poses.size(); i++)
      {
        const cv::Mat& landmarks = poses[i];
        // the box of all the points, enlarged
        double min_x, max_x, min_y, max_y;
        cv::minMaxLoc(landmarks.col(0), &min_x, &max_x);
        cv::minMaxLoc(landmarks.col(1), &min_y, &max_y);
        opencv_apps::msg::Detection detection_msg;
        detection_msg.label = "person";
        detection_msg.id = -1;
        detection_msg.score = confidences[i];
        detection_msg.rect.x = (min_x + max_x) / 2;
        detection_msg.rect.y = (min_y + max_y) / 2;
        detection_msg.rect.width = (max_x - min_x) * PERSON_BOX_ENLARGE_FACTOR;
        detection_msg.rect.height = (max_y - min_y) * PERSON_BOX_ENLARGE_FACTOR;
        for (int j = 0; j < NUM_LANDMARKS; j++)
        {
          opencv_apps::msg::Point2D point_msg;
          point_msg.x = landmarks.at<float>(j, 0);
          point_msg.y = landmarks.at<float>(j, 1);
          detection_msg.corners.push_back(point_msg);
        }
        detections_msg.detections.push_back(detection_msg);

        // draw the box, and the skeleton of the landmarks of enough presence
        cv::rectangle(frame,
                      cv::Rect2d(detection_msg.rect.x - detection_msg.rect.width / 2,
                                 detection_msg.rect.y - detection_msg.rect.height / 2, detection_msg.rect.width,
                                 detection_msg.rect.height),
                      cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, cv::format("person %.2f", confidences[i]),
                    cv::Point(cvRound(detection_msg.rect.x - detection_msg.rect.width / 2),
                              std::max(15, cvRound(detection_msg.rect.y - detection_msg.rect.height / 2) - 5)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        auto visible = [&](int j) { return landmarks.at<float>(j, 4) > LANDMARK_PRESENCE_THRESHOLD; };
        auto landmark = [&](int j) {
          return cv::Point(cvRound(landmarks.at<float>(j, 0)), cvRound(landmarks.at<float>(j, 1)));
        };
        for (const auto& line : SKELETON)
          if (visible(line[0]) && visible(line[1]))
            cv::line(frame, landmark(line[0]), landmark(line[1]), cv::Scalar(255, 255, 255), 2);
        for (int j = 0; j < NUM_LANDMARKS; j++)
          if (visible(j))
            cv::circle(frame, landmark(j), 3, cv::Scalar(0, 0, 255), cv::FILLED);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Pose estimation", frame);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNPoseEstimationNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNPoseEstimationNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNPoseEstimationNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_pose_estimation", options)
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
    anchors_ = personDetectionAnchors();

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    person_detection_model_ = declare_parameter(
        "person_detection_model", std::string(""),
        paramDescriptor("Path or URL of the MediaPipe person detection model (onnx), empty for "
                        "person_detection_mediapipe_2023mar.onnx of opencv_zoo. The URL is downloaded to "
                        "~/.ros/opencv_apps/models/ at the first time"));
    pose_estimation_model_ = declare_parameter(
        "pose_estimation_model", std::string(""),
        paramDescriptor("Path or URL of the MediaPipe pose estimation model (onnx), empty for "
                        "pose_estimation_mediapipe_2023mar.onnx of opencv_zoo. The URL is downloaded to "
                        "~/.ros/opencv_apps/models/ at the first time"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    confidence_threshold_ = declare_parameter(
        "confidence_threshold", 0.5,
        paramDescriptor("Threshold of the confidence to detect the persons and to estimate their poses", 0.0, 1.0));
    nms_threshold_ = declare_parameter(
        "nms_threshold", 0.3,
        paramDescriptor("Threshold of the overlap of the persons to suppress, by non maximum suppression", 0.0, 1.0));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNPoseEstimationNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::DetectionArrayStamped>("~/detections", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNPoseEstimationNodelet)
