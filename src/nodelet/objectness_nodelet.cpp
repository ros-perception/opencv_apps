// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Yoshiki Obinata.
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
 *   * Neither the name of the Kei Okada nor the names of its
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

// https://github.com/opencv/opencv_contrib/blob/4.x/modules/saliency/samples/computeSaliency.cpp
/**
 * Objectness proposals with BING (cv::saliency::ObjectnessBING)
 */

#include "opencv_apps/nodelet.h"
#include "opencv_apps/dnn.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/saliency.hpp>

#include <opencv_apps/msg/rect.hpp>
#include <opencv_apps/msg/rect_array_stamped.hpp>

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
// the trained model of BING in opencv_contrib, the commit (4.6.0) is fixed not to change the files
static const char* const BING_TRAINED_MODEL_URL =
    "https://raw.githubusercontent.com/opencv/opencv_contrib/db16caf6ceee76b43b94c846be276e92a43e9700/"
    "modules/saliency/samples/ObjectnessTrainedModel/";
// ObjectnessBING reads the models of all the color spaces in the training path, ObjNessB<base>W<W><color>.*
static const char* const BING_TRAINED_MODEL_FILES[] = {
  "ObjNessB2W8MAXBGR.idx.yml.gz", "ObjNessB2W8MAXBGR.wS1.yml.gz", "ObjNessB2W8MAXBGR.wS2.yml.gz",
  "ObjNessB2W8HSV.idx.yml.gz",    "ObjNessB2W8HSV.wS1.yml.gz",    "ObjNessB2W8HSV.wS2.yml.gz",
  "ObjNessB2W8I.idx.yml.gz",      "ObjNessB2W8I.wS1.yml.gz",      "ObjNessB2W8I.wS2.yml.gz",
};

class ObjectnessNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::RectArrayStamped>::SharedPtr msg_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string training_path_;
  int nss_;
  int max_objectness_;

  std::mutex mutex_;
  cv::Ptr<cv::saliency::ObjectnessBING> objectness_;
  bool need_objectness_update_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "training_path")
      {
        training_path_ = parameter.as_string();
        need_objectness_update_ = true;
      }
      else if (parameter.get_name() == "nss")
        nss_ = parameter.as_int();
      else if (parameter.get_name() == "max_objectness")
        max_objectness_ = parameter.as_int();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // directory of the trained model, the files are downloaded to ~/.ros/opencv_apps/models/ when it is empty
  std::string trainingPath()
  {
    if (!training_path_.empty())
      return training_path_;
    std::string path;
    for (const char* file : BING_TRAINED_MODEL_FILES)
    {
      path = modelFile(std::string(BING_TRAINED_MODEL_URL) + file, get_logger());
      if (path.empty())
        return path;
    }
    return path.substr(0, path.rfind('/'));
  }

  // (re)create ObjectnessBING with the parameters, called with mutex_
  void updateObjectness()
  {
    if (!need_objectness_update_)
      return;
    need_objectness_update_ = false;
    objectness_.release();
    std::string training_path = trainingPath();
    if (training_path.empty())
      return;
    RCLCPP_INFO(get_logger(), "BING training path: %s", training_path.c_str());
    objectness_ = cv::saliency::ObjectnessBING::create();
    objectness_->setTrainingPath(training_path);
    // computeSaliency() writes the boxes to <BBResDir>/BBoxes*/bb.txt for each image, an invalid directory not to
    // write them (the default is the root directory)
    objectness_->setBBResDir("/dev/null");
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
      // Convert the image into something opencv can handle, BING requires 3 channels (e.g. of mono8 images)
      cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      if (frame.empty())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Empty image is given");
        return;
      }

      // Messages
      opencv_apps::msg::RectArrayStamped rects_msg;
      rects_msg.header = msg->header;
      rects_msg.header.frame_id = input_frame_from_msg;

      // Do the work
      // each box is (minX, minY, maxX, maxY), in descending order of the objectness
      std::vector<cv::Vec4i> boxes;
      int max_objectness;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        updateObjectness();
        if (!objectness_)
        {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                                "BING trained model is not loaded, training_path: %s", training_path_.c_str());
          return;
        }
        objectness_->setNSS(nss_);
        if (!objectness_->computeSaliency(frame, boxes))
          boxes.clear();
        max_objectness = max_objectness_;
      }

      for (size_t i = 0; i < std::min(boxes.size(), static_cast<size_t>(max_objectness)); i++)
      {
        const cv::Vec4i& b = boxes[i];
        opencv_apps::msg::Rect rect_msg;
        rect_msg.x = (b[0] + b[2]) / 2.0;
        rect_msg.y = (b[1] + b[3]) / 2.0;
        rect_msg.width = b[2] - b[0];
        rect_msg.height = b[3] - b[1];
        rects_msg.rects.push_back(rect_msg);
      }
      // draw the most likely one at the last
      for (size_t i = rects_msg.rects.size(); i-- > 0;)
      {
        const cv::Vec4i& b = boxes[i];
        cv::rectangle(frame, cv::Point(b[0], b[1]), cv::Point(b[2], b[3]),
                      i == 0 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
      }

      //-- Show what you got
      if (debug_view_)
      {
        cv::imshow("Objectness", frame);
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(rects_msg);
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
      cam_sub_ = subscribeCamera("image", queue_size_, &ObjectnessNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &ObjectnessNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit ObjectnessNodelet(const rclcpp::NodeOptions& options) : Nodelet("objectness", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    need_objectness_update_ = true;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    training_path_ = declare_parameter(
        "training_path", std::string(""),
        paramDescriptor("Directory of the BING trained model (ObjNessB2W8*.yml.gz), empty to download "
                        "opencv_contrib/modules/saliency/samples/ObjectnessTrainedModel to "
                        "~/.ros/opencv_apps/models/ at the first time"));
    nss_ = declare_parameter("nss", 2, paramDescriptor("Size for non-maximal suppress", 1, 255));
    max_objectness_ = declare_parameter("max_objectness", 10,
                                        paramDescriptor("Maximum number of objectness to be detected", 1, 10000));
    on_set_parameters_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&ObjectnessNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::RectArrayStamped>("~/rects", 1);

    onInitPostProcess();
  }
};
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::ObjectnessNodelet)
