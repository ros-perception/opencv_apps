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

// https://github.com/opencv/opencv/blob/4.x/samples/dnn/face_detect.cpp
/**
 * Face recognition with YuNet (cv::FaceDetectorYN) and SFace (cv::FaceRecognizerSF),
 * https://github.com/opencv/opencv_zoo/tree/main/models/face_recognition_sface
 *
 * The faces trained with the ~train service are saved to data_dir/<label>/<label>_<time>.png as the aligned face
 * images of SFace, and loaded at the start.
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <opencv_apps/msg/face.hpp>
#include <opencv_apps/msg/face_array.hpp>
#include <opencv_apps/msg/face_array_stamped.hpp>
#include <opencv_apps/srv/face_recognition_train.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace opencv_apps
{
class DNNFaceRecognitionNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::FaceArrayStamped>::SharedPtr msg_pub_;
  rclcpp::Service<opencv_apps::srv::FaceRecognitionTrain>::SharedPtr train_srv_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string detection_model_;
  std::string recognition_model_;
  std::string backend_;
  double score_threshold_;
  double match_threshold_;
  std::string data_dir_;

  std::mutex mutex_;
  cv::Ptr<cv::FaceDetectorYN> detector_;
  cv::Ptr<cv::FaceRecognizerSF> recognizer_;
  bool need_model_update_;
  bool need_data_update_;
  // label and feature of the trained faces
  std::vector<std::pair<std::string, cv::Mat> > features_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "detection_model")
        detection_model_ = parameter.as_string();
      else if (parameter.get_name() == "recognition_model")
        recognition_model_ = parameter.as_string();
      else if (parameter.get_name() == "backend")
        backend_ = parameter.as_string();
      else if (parameter.get_name() == "score_threshold")
      {
        score_threshold_ = parameter.as_double();
        if (detector_)
          detector_->setScoreThreshold(static_cast<float>(score_threshold_));
        continue;
      }
      else if (parameter.get_name() == "match_threshold")
      {
        match_threshold_ = parameter.as_double();
        continue;
      }
      else if (parameter.get_name() == "data_dir")
      {
        data_dir_ = parameter.as_string();
        need_data_update_ = true;
        continue;
      }
      else
        continue;
      need_model_update_ = true;
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  fs::path dataDir() const
  {
    const char* home = std::getenv("HOME");
    if (home != nullptr && (data_dir_ == "~" || data_dir_.compare(0, 2, "~/") == 0))
      return fs::path(std::string(home) + data_dir_.substr(1));
    return fs::path(data_dir_);
  }

  // (re)create the models and load the trained faces when the parameters are changed, or retry when the models
  // failed to load, called with mutex_
  bool updateModels()
  {
    if (need_model_update_ || !detector_ || !recognizer_)
    {
      need_model_update_ = false;
      need_data_update_ = true;
      detector_.release();
      recognizer_.release();
      int backend, target;
      if (!dnnBackend(backend_, backend, target, get_logger()))
        return false;
      std::string detection_model =
          modelFile(detection_model_.empty() ? YUNET_MODEL_URL : detection_model_, get_logger());
      std::string recognition_model =
          modelFile(recognition_model_.empty() ? SFACE_MODEL_URL : recognition_model_, get_logger());
      if (detection_model.empty() || recognition_model.empty())
        return false;
      detector_ = cv::FaceDetectorYN::create(detection_model, "", cv::Size(320, 320),
                                             static_cast<float>(score_threshold_), 0.3f, 5000, backend, target);
      recognizer_ = cv::FaceRecognizerSF::create(recognition_model, "", backend, target);
    }
    if (!detector_ || !recognizer_)
      return false;
    if (need_data_update_)
    {
      need_data_update_ = false;
      loadData();
    }
    return true;
  }

  // load data_dir/<label>/*.png, called with mutex_
  void loadData()
  {
    features_.clear();
    fs::path data_dir = dataDir();
    std::error_code ec;
    if (!fs::is_directory(data_dir, ec))
    {
      RCLCPP_INFO(get_logger(), "No trained faces in %s", data_dir.c_str());
      return;
    }
    for (const fs::directory_entry& label_dir : fs::directory_iterator(data_dir, ec))
    {
      // error_code not to throw, e.g. for a broken symbolic link
      if (!label_dir.is_directory(ec))
        continue;
      for (const fs::directory_entry& file : fs::directory_iterator(label_dir.path(), ec))
      {
        cv::Mat aligned_face = cv::imread(file.path().string());
        if (aligned_face.empty())
          continue;
        features_.push_back(std::make_pair(label_dir.path().filename().string(), feature(aligned_face)));
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu trained faces from %s", features_.size(), data_dir.c_str());
  }

  cv::Mat feature(const cv::Mat& aligned_face)
  {
    cv::Mat feature;
    recognizer_->feature(aligned_face, feature);
    return feature.clone();
  }

  // detected faces, each row is the face rectangle, 5 landmarks and the score, called with mutex_
  cv::Mat detect(const cv::Mat& image)
  {
    cv::Mat faces;
    if (detector_->getInputSize() != image.size())
      detector_->setInputSize(image.size());
    detector_->detect(image, faces);
    return faces;
  }

  // label and score of the most similar trained face, called with mutex_
  std::pair<std::string, double> match(const cv::Mat& feature)
  {
    std::pair<std::string, double> best("", 0.0);
    for (const std::pair<std::string, cv::Mat>& trained : features_)
    {
      double score = recognizer_->match(feature, trained.second, cv::FaceRecognizerSF::FR_COSINE);
      if (best.first.empty() || score > best.second)
        best = std::make_pair(trained.first, score);
    }
    return best;
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
      opencv_apps::msg::FaceArrayStamped faces_msg;
      faces_msg.header = msg->header;

      std::lock_guard<std::mutex> lock(mutex_);
      if (!updateModels())
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Models are not loaded: %s, %s",
                              detection_model_.c_str(), recognition_model_.c_str());
        return;
      }

      // Do the work, the faces are aligned from the frame before drawing the results on it
      cv::Mat faces = detect(frame);
      std::vector<cv::Rect> face_rects;
      for (int i = 0; i < faces.rows; i++)
      {
        cv::Mat aligned_face;
        recognizer_->alignCrop(frame, faces.row(i), aligned_face);
        std::pair<std::string, double> matched = match(feature(aligned_face));

        cv::Rect face(cvRound(faces.at<float>(i, 0)), cvRound(faces.at<float>(i, 1)), cvRound(faces.at<float>(i, 2)),
                      cvRound(faces.at<float>(i, 3)));
        opencv_apps::msg::Face face_msg;
        face_msg.face.x = face.x + face.width / 2.0;
        face_msg.face.y = face.y + face.height / 2.0;
        face_msg.face.width = face.width;
        face_msg.face.height = face.height;
        for (int j = 0; j < 2; j++)
        {
          opencv_apps::msg::Rect eye_msg;
          eye_msg.x = faces.at<float>(i, 4 + j * 2);
          eye_msg.y = faces.at<float>(i, 5 + j * 2);
          eye_msg.width = face.width / 4.0;
          eye_msg.height = face.height / 4.0;
          face_msg.eyes.push_back(eye_msg);
        }
        // the label is empty for unknown faces
        face_msg.label = (matched.second >= match_threshold_) ? matched.first : std::string();
        face_msg.confidence = matched.second;
        faces_msg.faces.push_back(face_msg);
        face_rects.push_back(face);
      }
      for (size_t i = 0; i < face_rects.size(); i++)
      {
        const opencv_apps::msg::Face& face_msg = faces_msg.faces[i];
        cv::Scalar color = face_msg.label.empty() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        cv::rectangle(frame, face_rects[i], color, 2);
        cv::putText(frame,
                    cv::format("%s (%.2f)", face_msg.label.empty() ? "unknown" : face_msg.label.c_str(),
                               face_msg.confidence),
                    cv::Point(face_rects[i].x, face_rects[i].y - 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Face recognition", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(faces_msg);
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

  // train the face of each image, in the rect if given (x-y is center), and save its aligned face image
  void trainCallback(const std::shared_ptr<opencv_apps::srv::FaceRecognitionTrain::Request> req,
                     std::shared_ptr<opencv_apps::srv::FaceRecognitionTrain::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    res->ok = false;
    try
    {
      if (req->images.size() != req->labels.size() || (!req->rects.empty() && req->rects.size() != req->images.size()))
      {
        res->error = "the sizes of images, rects (if given) and labels are different";
        return;
      }
      // the label is the name of the directory in data_dir
      for (const std::string& label : req->labels)
      {
        if (label.empty() || label == "." || label == ".." || label.find('/') != std::string::npos)
        {
          res->error = "invalid label '" + label + "', it is used as a directory name";
          return;
        }
      }
      if (!updateModels())
      {
        res->error = "models are not loaded";
        return;
      }
      // find the faces and their features in all the images first, not to train a part of them
      std::vector<cv::Mat> aligned_faces;
      std::vector<cv::Mat> features;
      for (size_t i = 0; i < req->images.size(); i++)
      {
        cv::Mat image = cv_bridge::toCvCopy(req->images[i], sensor_msgs::image_encodings::BGR8)->image;
        cv::Rect rect(0, 0, image.cols, image.rows);
        if (!req->rects.empty())
        {
          const opencv_apps::msg::Rect& r = req->rects[i];
          rect = cv::Rect(cvRound(r.x - r.width / 2), cvRound(r.y - r.height / 2), cvRound(r.width), cvRound(r.height));
        }
        // the face overlapping the rect most
        cv::Mat faces = detect(image);
        int best = -1;
        int best_area = 0;
        for (int j = 0; j < faces.rows; j++)
        {
          cv::Rect face(cvRound(faces.at<float>(j, 0)), cvRound(faces.at<float>(j, 1)), cvRound(faces.at<float>(j, 2)),
                        cvRound(faces.at<float>(j, 3)));
          int area = (face & rect).area();
          if (area > best_area)
          {
            best = j;
            best_area = area;
          }
        }
        if (best < 0)
        {
          res->error = "no face is found in the image " + std::to_string(i);
          return;
        }
        cv::Mat aligned_face;
        recognizer_->alignCrop(image, faces.row(best), aligned_face);
        aligned_faces.push_back(aligned_face);
        features.push_back(feature(aligned_face));
      }
      // save the faces, and remove the saved ones if failed, as they are loaded at the next start
      std::vector<fs::path> saved_files;
      for (size_t i = 0; i < aligned_faces.size() && res->error.empty(); i++)
      {
        fs::path label_dir = dataDir() / req->labels[i];
        std::error_code ec;
        fs::create_directories(label_dir, ec);
        long long now =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        fs::path file = label_dir / (req->labels[i] + "_" + std::to_string(now) + ".png");
        if (ec || !cv::imwrite(file.string(), aligned_faces[i]))
          res->error = "failed to save " + file.string() + (ec ? ": " + ec.message() : std::string());
        else
          saved_files.push_back(file);
      }
      if (!res->error.empty())
      {
        for (const fs::path& file : saved_files)
        {
          std::error_code ec;
          fs::remove(file, ec);
        }
        return;
      }
      for (size_t i = 0; i < aligned_faces.size(); i++)
        features_.push_back(std::make_pair(req->labels[i], features[i]));
      RCLCPP_INFO(get_logger(), "Trained %zu faces, %zu faces in total", req->images.size(), features_.size());
      res->ok = true;
    }
    catch (cv_bridge::Exception& e)
    {
      res->error = std::string("error at converting the image: ") + e.what();
    }
    catch (cv::Exception& e)
    {
      res->error = "error at training: " + e.err;
    }
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &DNNFaceRecognitionNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &DNNFaceRecognitionNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit DNNFaceRecognitionNodelet(const rclcpp::NodeOptions& options) : Nodelet("dnn_face_recognition", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    need_model_update_ = true;
    need_data_update_ = true;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    detection_model_ =
        declare_parameter("detection_model", std::string(""),
                          paramDescriptor("Path or URL of the YuNet face detection model (onnx), empty for the "
                                          "default for this OpenCV version. The URL is downloaded to "
                                          "~/.ros/opencv_apps/models/ at the first time"));
    recognition_model_ =
        declare_parameter("recognition_model", std::string(""),
                          paramDescriptor("Path or URL of the SFace face recognition model (onnx), empty for the "
                                          "default. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first "
                                          "time"));
    backend_ = declare_parameter("backend", std::string("auto"), paramDescriptor(DNN_BACKEND_DESCRIPTION));
    score_threshold_ = declare_parameter("score_threshold", 0.9,
                                         paramDescriptor("Threshold of the score to detect the faces", 0.0, 1.0));
    match_threshold_ = declare_parameter(
        "match_threshold", 0.363,
        paramDescriptor("Threshold of the cosine similarity to recognize the face as the trained one, 0.363 is "
                        "suggested by SFace",
                        -1.0, 1.0));
    data_dir_ = declare_parameter("data_dir", std::string("~/.ros/opencv_apps/dnn_face_data"),
                                  paramDescriptor("Directory to save and load the trained faces"));
    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&DNNFaceRecognitionNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::FaceArrayStamped>("~/output", 1);
    train_srv_ = create_service<opencv_apps::srv::FaceRecognitionTrain>(
        "~/train",
        std::bind(&DNNFaceRecognitionNodelet::trainCallback, this, std::placeholders::_1, std::placeholders::_2));

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::DNNFaceRecognitionNodelet)
