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
 * rotated so that the person stands upright. As MediaPipe, the persons are tracked by the auxiliary landmarks of
 * their poses in the previous image, and the person detection runs every detection_interval images to find the new
 * persons, or when no person is tracked. The models need OpenCV 4.9 or later (OpenCV 4.6 cannot read them).
 *
 * ~skeletons has a Skeleton for each person: label "person", id the id while tracked, score the confidence of the
 * pose, rect the bounding box of the landmarks, and keypoints the 33 landmarks in the image coordinates with their
 * visibility and presence, in the order of MediaPipe (LANDMARK_NAMES).
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv_apps/msg/keypoint.hpp>
#include <opencv_apps/msg/skeleton.hpp>
#include <opencv_apps/msg/skeleton_array_stamped.hpp>

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
const char* const LANDMARK_NAMES[NUM_LANDMARKS] = {
  "nose",        "left_eye_inner",  "left_eye",        "left_eye_outer", "right_eye_inner",
  "right_eye",   "right_eye_outer", "left_ear",        "right_ear",      "mouth_left",
  "mouth_right", "left_shoulder",   "right_shoulder",  "left_elbow",     "right_elbow",
  "left_wrist",  "right_wrist",     "left_pinky",      "right_pinky",    "left_index",
  "right_index", "left_thumb",      "right_thumb",     "left_hip",       "right_hip",
  "left_knee",   "right_knee",      "left_ankle",      "right_ankle",    "left_heel",
  "right_heel",  "left_foot_index", "right_foot_index"
};
const int NUM_POSE_POINTS = 39;
// the box of the landmarks is enlarged by this factor, as opencv_zoo
const double PERSON_BOX_ENLARGE_FACTOR = 1.25;
// the region of the tracked person is enlarged by this factor from the auxiliary landmarks, as MediaPipe
const float TRACKING_ROI_SCALE = 1.25f;
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
  rclcpp::Publisher<opencv_apps::msg::SkeletonArrayStamped>::SharedPtr msg_pub_;

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
  bool tracking_;
  int detection_interval_;

  std::mutex mutex_;
  cv::dnn::Net person_detection_;
  cv::dnn::Net pose_estimation_;
  std::vector<cv::Point2f> anchors_;
  bool need_model_update_;
  bool read_failed_;  // the models could not be read, not retried until the parameters change

  // a person detected by the person detection model, or tracked by the auxiliary landmarks of the previous pose
  struct Person
  {
    float score;
    cv::Point2f mid_hip;    // the center of the full body
    cv::Point2f full_body;  // the point on the circle of the full body
    int id;                 // id of the tracked person, or -1 for the detected one
  };
  // a pose estimated for a person
  struct Pose
  {
    int id;
    cv::Mat landmarks;
    float confidence;
    cv::Rect2d box;  // the box of the landmarks
  };

  std::vector<Person> tracked_persons_;
  int next_id_;
  int images_since_detection_;

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
      else if (parameter.get_name() == "tracking")
        tracking_ = parameter.as_bool();
      else if (parameter.get_name() == "detection_interval")
        detection_interval_ = parameter.as_int();
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
      candidates.push_back(Person{ score, point(delta + 4, anchors_[i]), point(delta + 6, anchors_[i]), -1 });
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

  static cv::Rect2d landmarksBox(const cv::Mat& landmarks)
  {
    double min_x, max_x, min_y, max_y;
    cv::Mat points = landmarks.rowRange(0, NUM_LANDMARKS);
    cv::minMaxLoc(points.col(0), &min_x, &max_x);
    cv::minMaxLoc(points.col(1), &min_y, &max_y);
    return cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
  }

  static double iou(const cv::Rect2d& a, const cv::Rect2d& b)
  {
    double intersection = (a & b).area();
    return intersection > 0 ? intersection / (a.area() + b.area() - intersection) : 0.0;
  }

  // estimate the poses of the tracked persons and of the detected persons, and keep one pose for each person,
  // then track the persons by the auxiliary landmarks of their poses, called with mutex_
  std::vector<Pose> estimatePoses(const cv::Mat& frame)
  {
    std::vector<Pose> candidates;
    auto estimate = [&](const std::vector<Person>& persons) {
      bool lost = false;
      for (const Person& person : persons)
      {
        Pose pose;
        pose.id = person.id;
        if (!estimatePose(frame, person, pose.landmarks, pose.confidence))
        {
          lost = true;
          continue;
        }
        pose.box = landmarksBox(pose.landmarks);
        candidates.push_back(pose);
      }
      return lost;
    };
    // the person detection runs also when a tracked person is lost, to find the person again in this image
    bool lost = tracking_ && estimate(tracked_persons_);
    if (!tracking_ || tracked_persons_.empty() || lost || ++images_since_detection_ >= detection_interval_)
    {
      images_since_detection_ = 0;
      estimate(detectPersons(frame));
    }
    // the poses of the tracked persons first, then of the new persons, in the order of the confidence, which do
    // not overlap the poses kept already
    std::stable_sort(candidates.begin(), candidates.end(), [](const Pose& a, const Pose& b) {
      return (a.id >= 0) != (b.id >= 0) ? a.id >= 0 : a.confidence > b.confidence;
    });
    std::vector<Pose> poses;
    tracked_persons_.clear();
    for (Pose& pose : candidates)
    {
      if (std::any_of(poses.begin(), poses.end(),
                      [&](const Pose& kept) { return iou(pose.box, kept.box) > nms_threshold_; }))
        continue;
      if (!tracking_)
        pose.id = -1;
      else if (pose.id < 0)
        pose.id = next_id_++;
      poses.push_back(pose);
      if (tracking_)
      {
        // the auxiliary landmarks 33 and 34 are the mid hip and the point on the circle of the full body, as the
        // keypoints of the person detection, and the circle is enlarged by 1.25 as pose_landmarks_to_roi of MediaPipe
        const cv::Mat& l = pose.landmarks;
        cv::Point2f mid_hip(l.at<float>(33, 0), l.at<float>(33, 1));
        cv::Point2f full_body(l.at<float>(34, 0), l.at<float>(34, 1));
        tracked_persons_.push_back(
            Person{ pose.confidence, mid_hip, mid_hip + (full_body - mid_hip) * TRACKING_ROI_SCALE, pose.id });
      }
    }
    return poses;
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
      opencv_apps::msg::SkeletonArrayStamped skeletons_msg;
      skeletons_msg.header = msg->header;

      // Do the work
      std::vector<Pose> poses;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Models are not loaded");
          return;
        }
        poses = estimatePoses(frame);
      }

      for (const Pose& pose : poses)
      {
        const cv::Mat& landmarks = pose.landmarks;
        // the box of the landmarks, enlarged
        double min_x = pose.box.x, max_x = pose.box.br().x, min_y = pose.box.y, max_y = pose.box.br().y;
        opencv_apps::msg::Skeleton skeleton_msg;
        skeleton_msg.label = "person";
        skeleton_msg.id = pose.id;
        skeleton_msg.score = pose.confidence;
        skeleton_msg.rect.x = (min_x + max_x) / 2;
        skeleton_msg.rect.y = (min_y + max_y) / 2;
        skeleton_msg.rect.width = (max_x - min_x) * PERSON_BOX_ENLARGE_FACTOR;
        skeleton_msg.rect.height = (max_y - min_y) * PERSON_BOX_ENLARGE_FACTOR;
        for (int j = 0; j < NUM_LANDMARKS; j++)
        {
          opencv_apps::msg::Keypoint keypoint_msg;
          keypoint_msg.name = LANDMARK_NAMES[j];
          keypoint_msg.x = landmarks.at<float>(j, 0);
          keypoint_msg.y = landmarks.at<float>(j, 1);
          keypoint_msg.visibility = landmarks.at<float>(j, 3);
          keypoint_msg.presence = landmarks.at<float>(j, 4);
          skeleton_msg.keypoints.push_back(keypoint_msg);
        }
        skeletons_msg.skeletons.push_back(skeleton_msg);

        // draw the box, and the skeleton of the landmarks of enough presence
        cv::rectangle(frame,
                      cv::Rect2d(skeleton_msg.rect.x - skeleton_msg.rect.width / 2,
                                 skeleton_msg.rect.y - skeleton_msg.rect.height / 2, skeleton_msg.rect.width,
                                 skeleton_msg.rect.height),
                      cv::Scalar(0, 255, 0), 2);
        cv::putText(frame,
                    pose.id >= 0 ? cv::format("person %d %.2f", pose.id, pose.confidence) :
                                   cv::format("person %.2f", pose.confidence),
                    cv::Point(cvRound(skeleton_msg.rect.x - skeleton_msg.rect.width / 2),
                              std::max(15, cvRound(skeleton_msg.rect.y - skeleton_msg.rect.height / 2) - 5)),
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
      msg_pub_->publish(skeletons_msg);
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
    next_id_ = 0;
    images_since_detection_ = 0;

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
    tracking_ = declare_parameter(
        "tracking", true,
        paramDescriptor("Track the persons by their poses in the previous image, as MediaPipe, and run the person "
                        "detection every detection_interval images, otherwise every image"));
    detection_interval_ = declare_parameter(
        "detection_interval", 10,
        paramDescriptor("Interval of the person detection to find the new persons while tracking", 1, 1000));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNPoseEstimationNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::SkeletonArrayStamped>("~/skeletons", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNPoseEstimationNodelet)
