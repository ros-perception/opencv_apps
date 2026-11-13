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

// https://github.com/opencv/opencv_zoo/tree/main/models/handpose_estimation_mediapipe
// https://github.com/opencv/opencv_zoo/tree/main/models/palm_detection_mediapipe
/**
 * Hand pose estimation with MediaPipe Hands of opencv_zoo (Apache 2.0): the palms are detected by the palm detection
 * model, then the 21 landmarks of each hand are estimated by the hand pose estimation model, on the image rotated so
 * that the fingers point up. As MediaPipe, the hands are tracked by the palm landmarks of their poses in the previous
 * image, and the palm detection runs every detection_interval images to find the new hands, or when a tracked hand
 * is lost. The landmarks of the tracked hands are smoothed by One Euro filter, as MediaPipe. The hand pose estimation
 * model needs OpenCV 4.9 or later (OpenCV 4.6 cannot read it).
 *
 * ~skeletons has a Skeleton for each hand: label "left" or "right" (the handedness of the model, which assumes the
 * mirrored image of a front camera as MediaPipe), id the id while tracked, score the confidence of the hand, rect
 * the bounding box of the landmarks, and keypoints the 21 landmarks in the image coordinates, in the order of
 * MediaPipe (LANDMARK_NAMES), whose visibility and presence are the confidence of the hand.
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
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
namespace
{
// the models of https://github.com/opencv/opencv_zoo, the commit is fixed not to change the files
const char* const PALM_DETECTION_MODEL_URL =
    "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
    "models/palm_detection_mediapipe/palm_detection_mediapipe_2023feb.onnx";
const char* const HAND_POSE_ESTIMATION_MODEL_URL =
    "https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/"
    "models/handpose_estimation_mediapipe/handpose_estimation_mediapipe_2023feb.onnx";

const int PALM_DETECTION_INPUT_SIZE = 192;
const int HAND_POSE_ESTIMATION_INPUT_SIZE = 224;
const int NUM_LANDMARKS = 21;
const char* const LANDMARK_NAMES[NUM_LANDMARKS] = { "wrist",
                                                    "thumb_cmc",
                                                    "thumb_mcp",
                                                    "thumb_ip",
                                                    "thumb_tip",
                                                    "index_finger_mcp",
                                                    "index_finger_pip",
                                                    "index_finger_dip",
                                                    "index_finger_tip",
                                                    "middle_finger_mcp",
                                                    "middle_finger_pip",
                                                    "middle_finger_dip",
                                                    "middle_finger_tip",
                                                    "ring_finger_mcp",
                                                    "ring_finger_pip",
                                                    "ring_finger_dip",
                                                    "ring_finger_tip",
                                                    "pinky_mcp",
                                                    "pinky_pip",
                                                    "pinky_dip",
                                                    "pinky_tip" };
// the landmarks of the hand at the 7 keypoints of the palm detection (the palm base, the bases of the index, middle,
// ring and pinky fingers, and the two points of the thumb), to track the hand by its palm, as opencv_zoo
const int PALM_LANDMARK_IDS[] = { 0, 5, 9, 13, 17, 1, 2 };
const int NUM_PALM_KEYPOINTS = 7;
// the palm keypoints of the palm base and of the base of the middle finger, which give the rotation of the hand
const int PALM_BASE = 0;
const int MIDDLE_FINGER_BASE = 2;
// the region of the palm is shifted and enlarged by these factors to the region of the hand, as opencv_zoo
const double PALM_BOX_PRE_ENLARGE_FACTOR = 4;
const double PALM_BOX_SHIFT_Y = -0.4;
const double PALM_BOX_ENLARGE_FACTOR = 3;
// the box of the landmarks is shifted and enlarged by these factors to the box of the hand, as opencv_zoo
const double HAND_BOX_SHIFT_Y = -0.1;
const double HAND_BOX_ENLARGE_FACTOR = 1.65;

// the lines between the landmarks to draw the skeleton
const int SKELETON[][2] = { { 0, 1 },   { 1, 2 },   { 2, 3 },   { 3, 4 },   { 0, 5 },   { 5, 6 },   { 6, 7 },
                            { 7, 8 },   { 5, 9 },   { 9, 10 },  { 10, 11 }, { 11, 12 }, { 9, 13 },  { 13, 14 },
                            { 14, 15 }, { 15, 16 }, { 13, 17 }, { 0, 17 },  { 17, 18 }, { 18, 19 }, { 19, 20 } };

// the anchors (normalized centers) of the palm detection model, as SsdAnchorsCalculator of MediaPipe for the 192x192
// input: the layers of strides 8, 16, 16 and 16 have 2 anchors at each cell, and the layers of the same stride share
// the cells, so 2 and 6 anchors for the strides 8 and 16
std::vector<cv::Point2f> palmDetectionAnchors()
{
  std::vector<cv::Point2f> anchors;
  const int strides[][2] = { { 8, 2 }, { 16, 6 } };
  for (const auto& stride : strides)
  {
    int cells = PALM_DETECTION_INPUT_SIZE / stride[0];
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

// the box from the minimum to the maximum of the points, as opencv_zoo
cv::Rect2f pointsBox(const std::vector<cv::Point2f>& points)
{
  cv::Point2f min_point = points[0], max_point = points[0];
  for (const cv::Point2f& point : points)
  {
    min_point = cv::Point2f(std::min(min_point.x, point.x), std::min(min_point.y, point.y));
    max_point = cv::Point2f(std::max(max_point.x, point.x), std::max(max_point.y, point.y));
  }
  return cv::Rect2f(min_point, max_point);
}

// the point transformed by the affine matrix (2x3, CV_64F)
cv::Point2f transform(const cv::Mat& matrix, const cv::Point2f& point)
{
  return cv::Point2f(static_cast<float>(matrix.at<double>(0, 0) * point.x + matrix.at<double>(0, 1) * point.y +
                                        matrix.at<double>(0, 2)),
                     static_cast<float>(matrix.at<double>(1, 0) * point.x + matrix.at<double>(1, 1) * point.y +
                                        matrix.at<double>(1, 2)));
}
}  // namespace

class DNNHandPoseEstimationNodelet : public opencv_apps::Nodelet
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

  std::string palm_detection_model_;
  std::string hand_pose_estimation_model_;
  std::string backend_;
  double confidence_threshold_;
  double nms_threshold_;
  bool tracking_;
  int detection_interval_;
  bool smoothing_;
  double smoothing_min_cutoff_;
  double smoothing_beta_;
  double smoothing_derivative_cutoff_;

  std::mutex mutex_;
  cv::dnn::Net palm_detection_;
  cv::dnn::Net hand_pose_estimation_;
  std::vector<cv::Point2f> anchors_;
  bool need_model_update_;
  bool read_failed_;  // the models could not be read, not retried until the parameters change

  // a palm detected by the palm detection model, or tracked by the landmarks of the previous hand
  struct Palm
  {
    float score;
    cv::Rect2f box;
    std::vector<cv::Point2f> keypoints;  // NUM_PALM_KEYPOINTS
    int id;                              // id of the tracked hand, or -1 for the detected one
  };
  // a hand estimated for a palm
  struct Hand
  {
    int id;
    cv::Mat landmarks;  // NUM_LANDMARKS x (x, y, z) in the image coordinates
    float confidence;
    float handedness;  // 0 (left) .. 1 (right)
    cv::Rect2d box;    // the box of the landmarks
  };

  // One Euro filter of the landmarks of a tracked hand
  struct LandmarksFilter
  {
    double time;
    cv::Mat value;       // the smoothed landmarks, NUM_LANDMARKS x 2 (x, y), CV_64F
    cv::Mat raw_value;   // the landmarks of the previous image, to compute the velocity as MediaPipe
    cv::Mat derivative;  // in the scale of the hand per second
  };

  std::vector<Palm> tracked_palms_;
  std::map<int, LandmarksFilter> filters_;
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
      else if (parameter.get_name() == "smoothing")
        smoothing_ = parameter.as_bool();
      else if (parameter.get_name() == "smoothing_min_cutoff")
        smoothing_min_cutoff_ = parameter.as_double();
      else if (parameter.get_name() == "smoothing_beta")
        smoothing_beta_ = parameter.as_double();
      else if (parameter.get_name() == "smoothing_derivative_cutoff")
        smoothing_derivative_cutoff_ = parameter.as_double();
      else
      {
        if (parameter.get_name() == "palm_detection_model")
          palm_detection_model_ = parameter.as_string();
        else if (parameter.get_name() == "hand_pose_estimation_model")
          hand_pose_estimation_model_ = parameter.as_string();
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
    if (!need_model_update_ && ((!palm_detection_.empty() && !hand_pose_estimation_.empty()) || read_failed_))
      return !read_failed_;
    need_model_update_ = false;
    read_failed_ = false;
    palm_detection_ = cv::dnn::Net();
    hand_pose_estimation_ = cv::dnn::Net();
    int backend, target;
    if (!dnnBackend(backend_, backend, target, get_logger()))
      return false;
    std::string palm_detection_file = modelFile(
        palm_detection_model_.empty() ? std::string(PALM_DETECTION_MODEL_URL) : palm_detection_model_, get_logger());
    std::string hand_pose_estimation_file = modelFile(
        hand_pose_estimation_model_.empty() ? std::string(HAND_POSE_ESTIMATION_MODEL_URL) : hand_pose_estimation_model_,
        get_logger());
    if (palm_detection_file.empty() || hand_pose_estimation_file.empty())
      return false;
    try
    {
      palm_detection_ = cv::dnn::readNet(palm_detection_file);
      hand_pose_estimation_ = cv::dnn::readNet(hand_pose_estimation_file);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to read the models, which need OpenCV 4.9 or later (this is %s): %s",
                   CV_VERSION, e.what());
      palm_detection_ = cv::dnn::Net();
      hand_pose_estimation_ = cv::dnn::Net();
      read_failed_ = true;
      return false;
    }
    for (cv::dnn::Net* net : { &palm_detection_, &hand_pose_estimation_ })
    {
      net->setPreferableBackend(backend);
      net->setPreferableTarget(target);
    }
    return true;
  }

  // detect the palms, as palm_detection_mediapipe of opencv_zoo, called with mutex_
  std::vector<Palm> detectPalms(const cv::Mat& frame)
  {
    // the image in RGB, normalized to 0..1 and resized keeping the aspect ratio into the center of the input, padded
    // with 0 after the normalization, in NHWC
    const int input_size = PALM_DETECTION_INPUT_SIZE;
    double ratio = std::min(static_cast<double>(input_size) / frame.cols, static_cast<double>(input_size) / frame.rows);
    cv::Size resized_size(std::max(1, static_cast<int>(frame.cols * ratio)),
                          std::max(1, static_cast<int>(frame.rows * ratio)));
    cv::Point pad((input_size - resized_size.width) / 2, (input_size - resized_size.height) / 2);
    cv::Mat resized;
    cv::resize(frame, resized, resized_size);
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    cv::Mat input(input_size, input_size, CV_32FC3, cv::Scalar::all(0));
    cv::Mat roi = input(cv::Rect(pad, resized_size));
    resized.convertTo(roi, CV_32F, 1.0 / 255);
    const int shape[] = { 1, input_size, input_size, 3 };
    palm_detection_.setInput(cv::Mat(4, shape, CV_32F, input.ptr<float>()));
    std::vector<cv::Mat> outputs;
    palm_detection_.forward(outputs, palm_detection_.getUnconnectedOutLayersNames());
    // 1 x anchors x (4 box + 7 x 2 keypoints), and 1 x anchors x 1 score
    cv::Mat deltas = outputOfSize(outputs, anchors_.size() * 18);
    cv::Mat scores = outputOfSize(outputs, anchors_.size());
    if (deltas.empty() || scores.empty())
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                            "The outputs of the model are not of MediaPipe palm detection");
      return std::vector<Palm>();
    }
    deltas = deltas.reshape(1, static_cast<int>(anchors_.size()));
    scores = scores.reshape(1, static_cast<int>(anchors_.size()));

    // the normalized coordinates of the input to the image
    double scale = input_size / ratio;
    cv::Point2f offset(static_cast<float>(static_cast<int>(pad.x / ratio)),
                       static_cast<float>(static_cast<int>(pad.y / ratio)));
    auto point = [&](const float* delta, const cv::Point2f& anchor) {
      return cv::Point2f(static_cast<float>((delta[0] / input_size + anchor.x) * scale - offset.x),
                         static_cast<float>((delta[1] / input_size + anchor.y) * scale - offset.y));
    };
    std::vector<cv::Rect2d> boxes;
    std::vector<float> confidences;
    std::vector<Palm> candidates;
    for (int i = 0; i < deltas.rows; i++)
    {
      float score = 1.0f / (1.0f + std::exp(-std::max(-100.0f, std::min(100.0f, scores.at<float>(i)))));
      if (score < confidence_threshold_)
        continue;
      const float* delta = deltas.ptr<float>(i);
      cv::Point2f center = point(delta, anchors_[i]);
      cv::Point2f size(static_cast<float>(delta[2] / input_size * scale),
                       static_cast<float>(delta[3] / input_size * scale));
      cv::Rect2f box(center.x - size.x / 2, center.y - size.y / 2, size.x, size.y);
      boxes.push_back(box);
      confidences.push_back(score);
      Palm palm{ score, box, std::vector<cv::Point2f>(), -1 };
      for (int k = 0; k < NUM_PALM_KEYPOINTS; k++)
        palm.keypoints.push_back(point(delta + 4 + k * 2, anchors_[i]));
      candidates.push_back(palm);
    }
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, static_cast<float>(confidence_threshold_), static_cast<float>(nms_threshold_),
                      indices);
    std::vector<Palm> palms;
    for (int i : indices)
      palms.push_back(candidates[i]);
    return palms;
  }

  // the box shifted by shift_y of its height and enlarged by the factor, clipped by the image, and the image of the
  // box padded to the square of side (the diagonal for the rotation, or the longer side), as _cropAndPadFromPalm of
  // opencv_zoo, whose origin is returned as bias
  static cv::Mat cropAndPad(const cv::Mat& image, const cv::Rect2f& palm_box, double shift_y, double enlarge,
                            bool for_rotation, cv::Rect& box, cv::Point& bias)
  {
    cv::Point2f center(palm_box.x + palm_box.width / 2, palm_box.y + palm_box.height * (0.5f + shift_y));
    cv::Point2f half(static_cast<float>(palm_box.width * enlarge / 2),
                     static_cast<float>(palm_box.height * enlarge / 2));
    cv::Point top_left(static_cast<int>(center.x - half.x), static_cast<int>(center.y - half.y));
    cv::Point bottom_right(static_cast<int>(center.x + half.x), static_cast<int>(center.y + half.y));
    top_left.x = std::min(std::max(top_left.x, 0), image.cols);
    top_left.y = std::min(std::max(top_left.y, 0), image.rows);
    bottom_right.x = std::min(std::max(bottom_right.x, 0), image.cols);
    bottom_right.y = std::min(std::max(bottom_right.y, 0), image.rows);
    box = cv::Rect(top_left, bottom_right);
    cv::Mat cropped = image(box);
    int side = for_rotation ? static_cast<int>(std::sqrt(static_cast<double>(box.width) * box.width +
                                                         static_cast<double>(box.height) * box.height)) :
                              std::max(box.width, box.height);
    int left = (side - box.width) / 2, top = (side - box.height) / 2;
    cv::Mat padded;
    cv::copyMakeBorder(cropped, padded, top, side - box.height - top, left, side - box.width - left,
                       cv::BORDER_CONSTANT | cv::BORDER_ISOLATED, cv::Scalar::all(0));
    bias = box.tl() - cv::Point(left, top);
    return padded;
  }

  // estimate the landmarks of the hand of the palm, as handpose_estimation_mediapipe of opencv_zoo, called with mutex_
  bool estimateHand(const cv::Mat& frame, const Palm& palm, Hand& hand)
  {
    if (palm.box.width <= 0 || palm.box.height <= 0)
      return false;
    // the region around the palm, padded to rotate
    cv::Rect box;
    cv::Point bias;
    cv::Mat image = cropAndPad(frame, palm.box, 0, PALM_BOX_PRE_ENLARGE_FACTOR, true, box, bias);
    if (box.area() <= 0)
      return false;
    std::vector<cv::Point2f> keypoints;
    for (const cv::Point2f& keypoint : palm.keypoints)
      keypoints.push_back(keypoint - cv::Point2f(bias));

    // rotate the image so that the fingers point up, around the center of the region
    const cv::Point2f& p1 = keypoints[PALM_BASE];
    const cv::Point2f& p2 = keypoints[MIDDLE_FINGER_BASE];
    double radians = CV_PI / 2 - std::atan2(-(p2.y - p1.y), p2.x - p1.x);
    radians -= 2 * CV_PI * std::floor((radians + CV_PI) / (2 * CV_PI));
    double angle = radians * 180 / CV_PI;
    cv::Point2f box_center(static_cast<float>(box.x - bias.x + box.width / 2.0),
                           static_cast<float>(box.y - bias.y + box.height / 2.0));
    cv::Mat rotation = cv::getRotationMatrix2D(box_center, angle, 1.0);
    cv::Mat rotated;
    cv::warpAffine(image, rotated, rotation, image.size());
    // the region of the hand from the rotated keypoints of the palm
    std::vector<cv::Point2f> rotated_keypoints;
    for (const cv::Point2f& keypoint : keypoints)
      rotated_keypoints.push_back(transform(rotation, keypoint));
    cv::Rect2f rotated_palm_box = pointsBox(rotated_keypoints);
    cv::Rect hand_box;
    cv::Point unused;
    cv::Mat cropped =
        cropAndPad(rotated, rotated_palm_box, PALM_BOX_SHIFT_Y, PALM_BOX_ENLARGE_FACTOR, false, hand_box, unused);
    if (hand_box.area() <= 0)
      return false;

    // the input in NHWC, RGB, in 0..1
    const int input_size = HAND_POSE_ESTIMATION_INPUT_SIZE;
    cv::Mat input;
    cv::resize(cropped, input, cv::Size(input_size, input_size), 0, 0, cv::INTER_AREA);
    cv::cvtColor(input, input, cv::COLOR_BGR2RGB);
    input.convertTo(input, CV_32F, 1.0 / 255);
    const int shape[] = { 1, input_size, input_size, 3 };
    hand_pose_estimation_.setInput(cv::Mat(4, shape, CV_32F, input.ptr<float>()));
    // the landmarks, the confidence, the handedness and the world landmarks, in the order of the names as opencv_zoo
    std::vector<cv::Mat> outputs;
    hand_pose_estimation_.forward(outputs, hand_pose_estimation_.getUnconnectedOutLayersNames());
    if (outputs.size() != 4 || outputs[0].total() != NUM_LANDMARKS * 3 || outputs[1].total() != 1 ||
        outputs[2].total() != 1)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                            "The outputs of the model are not of MediaPipe hand pose estimation");
      return false;
    }
    hand.confidence = outputs[1].ptr<float>()[0];
    hand.handedness = outputs[2].ptr<float>()[0];
    if (hand.confidence < confidence_threshold_)
      return false;

    // the landmarks in the input to the image: scale to the region of the hand, rotate back around its center
    hand.landmarks = cv::Mat(NUM_LANDMARKS, 3, CV_32F, outputs[0].ptr<float>()).clone();
    double scale = static_cast<double>(std::max(hand_box.width, hand_box.height)) / input_size;
    cv::Mat inverse;
    cv::invertAffineTransform(rotation, inverse);
    cv::Point2f original_center =
        transform(inverse, cv::Point2f(static_cast<float>(hand_box.x + hand_box.width / 2.0),
                                       static_cast<float>(hand_box.y + hand_box.height / 2.0)));
    double cos_a = std::cos(radians), sin_a = std::sin(radians);
    for (int i = 0; i < NUM_LANDMARKS; i++)
    {
      float* landmark = hand.landmarks.ptr<float>(i);
      double x = (landmark[0] - input_size / 2.0) * scale;
      double y = (landmark[1] - input_size / 2.0) * scale;
      landmark[0] = static_cast<float>(x * cos_a - y * sin_a + original_center.x + bias.x);
      landmark[1] = static_cast<float>(x * sin_a + y * cos_a + original_center.y + bias.y);
      landmark[2] = static_cast<float>(landmark[2] * scale);
    }
    return true;
  }

  static cv::Rect2d landmarksBox(const cv::Mat& landmarks)
  {
    double min_x, max_x, min_y, max_y;
    cv::minMaxLoc(landmarks.col(0), &min_x, &max_x);
    cv::minMaxLoc(landmarks.col(1), &min_y, &max_y);
    return cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
  }

  // smooth the landmarks of the tracked hand by One Euro filter, whose velocity is computed from the landmarks of the
  // previous image and is in the scale of the hand as LandmarksSmoothingCalculator of MediaPipe, called with mutex_
  void smooth(Hand& hand, double time)
  {
    cv::Mat value;
    hand.landmarks.colRange(0, 2).convertTo(value, CV_64F);
    std::map<int, LandmarksFilter>::iterator it = filters_.find(hand.id);
    // restart the filter when the time goes back (e.g. the video loops) or jumps
    if (it == filters_.end() || time <= it->second.time || time - it->second.time > 1.0)
    {
      filters_[hand.id] = LandmarksFilter{ time, value, value.clone(), cv::Mat::zeros(value.size(), CV_64F) };
      return;
    }
    LandmarksFilter& filter = it->second;
    double dt = time - filter.time;
    double scale = std::max(1.0, (hand.box.width + hand.box.height) / 2);
    auto alpha = [dt](double cutoff) { return 1.0 / (1.0 + 1.0 / (2 * CV_PI * cutoff * dt)); };
    double derivative_alpha = alpha(smoothing_derivative_cutoff_);
    filter.derivative =
        derivative_alpha * (value - filter.raw_value) / (dt * scale) + (1 - derivative_alpha) * filter.derivative;
    for (int i = 0; i < value.rows; i++)
    {
      for (int j = 0; j < value.cols; j++)
      {
        double a = alpha(smoothing_min_cutoff_ + smoothing_beta_ * std::abs(filter.derivative.at<double>(i, j)));
        filter.value.at<double>(i, j) = a * value.at<double>(i, j) + (1 - a) * filter.value.at<double>(i, j);
        hand.landmarks.at<float>(i, j) = static_cast<float>(filter.value.at<double>(i, j));
      }
    }
    filter.raw_value = value;
    filter.time = time;
  }

  static double iou(const cv::Rect2d& a, const cv::Rect2d& b)
  {
    double intersection = (a & b).area();
    return intersection > 0 ? intersection / (a.area() + b.area() - intersection) : 0.0;
  }

  // estimate the hands of the tracked palms and of the detected palms, and keep one hand for each, then track the
  // hands by the palm landmarks of their poses, called with mutex_
  std::vector<Hand> estimateHands(const cv::Mat& frame, double time)
  {
    std::vector<Hand> candidates;
    auto estimate = [&](const std::vector<Palm>& palms) {
      bool lost = false;
      for (const Palm& palm : palms)
      {
        Hand hand;
        hand.id = palm.id;
        if (!estimateHand(frame, palm, hand))
        {
          lost = true;
          continue;
        }
        hand.box = landmarksBox(hand.landmarks);
        candidates.push_back(hand);
      }
      return lost;
    };
    // the palm detection runs also when a tracked hand is lost, to find the hand again in this image
    bool lost = tracking_ && estimate(tracked_palms_);
    if (!tracking_ || tracked_palms_.empty() || lost || ++images_since_detection_ >= detection_interval_)
    {
      images_since_detection_ = 0;
      estimate(detectPalms(frame));
    }
    // the tracked hands first, then the new hands, in the order of the confidence, which do not overlap the hands
    // kept already
    std::stable_sort(candidates.begin(), candidates.end(), [](const Hand& a, const Hand& b) {
      return (a.id >= 0) != (b.id >= 0) ? a.id >= 0 : a.confidence > b.confidence;
    });
    std::vector<Hand> hands;
    std::vector<cv::Rect2d> kept_boxes;  // the boxes of the hands kept, before the smoothing
    tracked_palms_.clear();
    for (Hand& hand : candidates)
    {
      if (std::any_of(kept_boxes.begin(), kept_boxes.end(),
                      [&](const cv::Rect2d& kept) { return iou(hand.box, kept) > nms_threshold_; }))
        continue;
      kept_boxes.push_back(hand.box);
      if (!tracking_)
        hand.id = -1;
      else if (hand.id < 0)
        hand.id = next_id_++;
      if (tracking_ && smoothing_)
      {
        smooth(hand, time);
        hand.box = landmarksBox(hand.landmarks);
      }
      hands.push_back(hand);
      if (tracking_)
      {
        // the palm of the next image from the landmarks at the keypoints of the palm detection
        Palm palm{ hand.confidence, cv::Rect2f(), std::vector<cv::Point2f>(), hand.id };
        for (int id : PALM_LANDMARK_IDS)
          palm.keypoints.push_back(cv::Point2f(hand.landmarks.at<float>(id, 0), hand.landmarks.at<float>(id, 1)));
        palm.box = pointsBox(palm.keypoints);
        tracked_palms_.push_back(palm);
      }
    }
    // the filters of the hands not tracked any more
    for (std::map<int, LandmarksFilter>::iterator it = filters_.begin(); it != filters_.end();)
    {
      if (std::none_of(hands.begin(), hands.end(), [&](const Hand& hand) { return hand.id == it->first; }))
        it = filters_.erase(it);
      else
        ++it;
    }
    return hands;
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
      std::vector<Hand> hands;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!updateModel())
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Models are not loaded");
          return;
        }
        hands = estimateHands(frame, rclcpp::Time(msg->header.stamp).seconds());
      }

      for (const Hand& hand : hands)
      {
        const cv::Mat& landmarks = hand.landmarks;
        // the box of the landmarks, shifted up and enlarged as opencv_zoo
        cv::Point2d center(hand.box.x + hand.box.width / 2, hand.box.y + hand.box.height * (0.5 + HAND_BOX_SHIFT_Y));
        opencv_apps::msg::Skeleton skeleton_msg;
        skeleton_msg.label = hand.handedness > 0.5 ? "right" : "left";
        skeleton_msg.id = hand.id;
        skeleton_msg.score = hand.confidence;
        skeleton_msg.rect.x = center.x;
        skeleton_msg.rect.y = center.y;
        skeleton_msg.rect.width = hand.box.width * HAND_BOX_ENLARGE_FACTOR;
        skeleton_msg.rect.height = hand.box.height * HAND_BOX_ENLARGE_FACTOR;
        for (int j = 0; j < NUM_LANDMARKS; j++)
        {
          opencv_apps::msg::Keypoint keypoint_msg;
          keypoint_msg.name = LANDMARK_NAMES[j];
          keypoint_msg.x = landmarks.at<float>(j, 0);
          keypoint_msg.y = landmarks.at<float>(j, 1);
          keypoint_msg.visibility = hand.confidence;
          keypoint_msg.presence = hand.confidence;
          skeleton_msg.keypoints.push_back(keypoint_msg);
        }
        skeletons_msg.skeletons.push_back(skeleton_msg);

        // draw the box, and the skeleton of the landmarks
        cv::Rect2d box(skeleton_msg.rect.x - skeleton_msg.rect.width / 2,
                       skeleton_msg.rect.y - skeleton_msg.rect.height / 2, skeleton_msg.rect.width,
                       skeleton_msg.rect.height);
        cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame,
                    hand.id >= 0 ? cv::format("%s %d %.2f", skeleton_msg.label.c_str(), hand.id, hand.confidence) :
                                   cv::format("%s %.2f", skeleton_msg.label.c_str(), hand.confidence),
                    cv::Point(cvRound(box.x), std::max(15, cvRound(box.y) - 5)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 0), 1);
        auto landmark = [&](int j) {
          return cv::Point(cvRound(landmarks.at<float>(j, 0)), cvRound(landmarks.at<float>(j, 1)));
        };
        for (const auto& line : SKELETON)
          cv::line(frame, landmark(line[0]), landmark(line[1]), cv::Scalar(255, 255, 255), 1);
        for (int j = 0; j < NUM_LANDMARKS; j++)
          cv::circle(frame, landmark(j), 2, cv::Scalar(0, 0, 255), cv::FILLED);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Hand pose estimation", frame);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNHandPoseEstimationNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNHandPoseEstimationNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNHandPoseEstimationNodelet(const rclcpp::NodeOptions& options)
    : Nodelet("dnn_hand_pose_estimation", options)
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
    anchors_ = palmDetectionAnchors();
    next_id_ = 0;
    images_since_detection_ = 0;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    palm_detection_model_ = declare_parameter(
        "palm_detection_model", std::string(""),
        paramDescriptor("Path or URL of the MediaPipe palm detection model (onnx), empty for "
                        "palm_detection_mediapipe_2023feb.onnx of opencv_zoo. The URL is downloaded to "
                        "~/.ros/opencv_apps/models/ at the first time"));
    hand_pose_estimation_model_ = declare_parameter(
        "hand_pose_estimation_model", std::string(""),
        paramDescriptor("Path or URL of the MediaPipe hand pose estimation model (onnx), empty for "
                        "handpose_estimation_mediapipe_2023feb.onnx of opencv_zoo. The URL is downloaded to "
                        "~/.ros/opencv_apps/models/ at the first time"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    confidence_threshold_ = declare_parameter(
        "confidence_threshold", 0.5,
        paramDescriptor("Threshold of the confidence to detect the palms and to estimate the hands", 0.0, 1.0));
    nms_threshold_ = declare_parameter(
        "nms_threshold", 0.3,
        paramDescriptor("Threshold of the overlap of the hands to suppress, by non maximum suppression", 0.0, 1.0));
    tracking_ = declare_parameter(
        "tracking", true,
        paramDescriptor("Track the hands by their poses in the previous image, as MediaPipe, and run the palm "
                        "detection every detection_interval images, otherwise every image"));
    detection_interval_ = declare_parameter(
        "detection_interval", 10,
        paramDescriptor("Interval of the palm detection to find the new hands while tracking", 1, 1000));
    smoothing_ = declare_parameter("smoothing", true,
                                   paramDescriptor("Smooth the landmarks of the tracked hands by One Euro filter"));
    smoothing_min_cutoff_ = declare_parameter(
        "smoothing_min_cutoff", 0.05,
        paramDescriptor("Minimum cutoff frequency (Hz) of One Euro filter, smaller is smoother when still", 0.001,
                        100.0));
    smoothing_beta_ = declare_parameter(
        "smoothing_beta", 80.0,
        paramDescriptor("Speed coefficient of One Euro filter, larger follows the fast motion with less lag", 0.0,
                        10000.0));
    smoothing_derivative_cutoff_ =
        declare_parameter("smoothing_derivative_cutoff", 1.0,
                          paramDescriptor("Cutoff frequency (Hz) of the velocity of One Euro filter", 0.001, 100.0));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNHandPoseEstimationNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::SkeletonArrayStamped>("~/skeletons", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNHandPoseEstimationNodelet)
