/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/lk_demo.cpp
/**
 * This is a demo of Lukas-Kanade optical flow lkdemo(),
 */

#include "opencv_apps/nodelet.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <std_srvs/srv/empty.hpp>
#include <opencv_apps/msg/flow_array_stamped.hpp>

namespace opencv_apps
{
class LKFlowNodelet : public opencv_apps::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  rclcpp::Publisher<opencv_apps::msg::FlowArrayStamped>::SharedPtr msg_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr initialize_points_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr delete_points_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_night_mode_service_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  int queue_size_;
  bool debug_view_;
  rclcpp::Time prev_stamp_;

  bool use_camera_info_;

  std::string window_name_;
  static bool need_config_update_;

  int MAX_COUNT;
  bool needToInit;
  bool nightMode;
  cv::Point2f point;
  bool addRemovePt;
  cv::Mat gray, prevGray;
  std::vector<cv::Point2f> points[2];
  size_t num_detected_points_;

  float quality_level_;
  int min_distance_;
  int block_size_;
  float harris_k_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
  {
    for (const rclcpp::Parameter& parameter : parameters)
    {
      if (parameter.get_name() == "use_camera_info")
        use_camera_info_ = parameter.as_bool();
      else if (parameter.get_name() == "quality_level")
        quality_level_ = parameter.as_double();
      else if (parameter.get_name() == "min_distance")
        min_distance_ = parameter.as_int();
      else if (parameter.get_name() == "block_size")
        block_size_ = parameter.as_int();
      else if (parameter.get_name() == "harris_k")
        harris_k_ = parameter.as_double();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
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
#if 0
  static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
  {
    if( event == CV_EVENT_LBUTTONDOWN )
    {
      point = Point2f((float)x, (float)y);
      addRemovePt = true;
    }
  }
#endif
  static void trackbarCallback(int /*unused*/, void* /*unused*/)
  {
    need_config_update_ = true;
  }

  void detectPoints(const cv::TermCriteria& termcrit, const cv::Size& sub_pix_win_size)
  {
    cv::goodFeaturesToTrack(gray, points[1], MAX_COUNT, quality_level_, min_distance_, cv::Mat(), block_size_, false,
                            harris_k_);
    if (!points[1].empty())
      cv::cornerSubPix(gray, points[1], sub_pix_win_size, cv::Size(-1, -1), termcrit);
    num_detected_points_ = points[1].size();
  }

  void doWork(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Messages
      opencv_apps::msg::FlowArrayStamped flows_msg;
      flows_msg.header = msg->header;

      if (debug_view_)
      {
        /// Create Trackbars for Thresholds
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        cv::createTrackbar("Min Distance", window_name_, &min_distance_, 100, trackbarCallback);
        cv::createTrackbar("Block Size", window_name_, &block_size_, 100, trackbarCallback);

        // cv::setMouseCallback( window_name_, onMouse, 0 );
        if (need_config_update_)
        {
          set_parameter(rclcpp::Parameter("quality_level", quality_level_));
          set_parameter(rclcpp::Parameter("min_distance", min_distance_));
          set_parameter(rclcpp::Parameter("block_size", block_size_));
          set_parameter(rclcpp::Parameter("harris_k", harris_k_));
          need_config_update_ = false;
        }
      }

      // Do the work
      cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
      cv::Size sub_pix_win_size(10, 10), win_size(31, 31);

      if (image.channels() > 1)
      {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      }
      else
      {
        image.copyTo(gray);
      }

      if (nightMode)
        image = cv::Scalar::all(0);

      if (needToInit)
      {
        // automatic initialization
        detectPoints(termcrit, sub_pix_win_size);
        addRemovePt = false;
      }
      else if (!points[0].empty())
      {
        std::vector<uchar> status;
        std::vector<float> err;
        if (prevGray.empty())
          gray.copyTo(prevGray);
        cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, win_size, 3, termcrit, 0, 0.001);
        // status has the size of points[1] here, and the loop compacts points[1] in place
        size_t i, k, n = status.size();
        for (i = k = 0; i < n; i++)
        {
          if (addRemovePt)
          {
            if (cv::norm(point - points[1][i]) <= 5)
            {
              addRemovePt = false;
              continue;
            }
          }

          opencv_apps::msg::Flow flow_msg;
          opencv_apps::msg::Point2D point_msg;
          opencv_apps::msg::Point2D velocity_msg;
          if (status[i])
          {
            cv::circle(image, points[1][i], 3, cv::Scalar(0, 255, 0), -1, 8);
            cv::line(image, points[1][i], points[0][i], cv::Scalar(0, 255, 0), 1, 8, 0);
            point_msg.x = points[1][i].x;
            point_msg.y = points[1][i].y;
            velocity_msg.x = points[1][i].x - points[0][i].x;
            velocity_msg.y = points[1][i].y - points[0][i].y;
          }
          flow_msg.point = point_msg;
          flow_msg.velocity = velocity_msg;
          flows_msg.status.push_back(status[i]);
          flows_msg.flow.push_back(flow_msg);

          // keep tracking only the points found in this frame, as the OpenCV sample does
          if (status[i])
            points[1][k++] = points[1][i];
        }
        points[1].resize(k);

        // detect the points again when most of them are lost, e.g. they went out of the view
        if (points[1].size() * 2 < num_detected_points_)
        {
          RCLCPP_DEBUG(get_logger(), "Only %zu of %zu points are tracked, detect the points again", points[1].size(),
                       num_detected_points_);
          detectPoints(termcrit, sub_pix_win_size);
        }
      }

      if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
      {
        std::vector<cv::Point2f> tmp;
        tmp.push_back(point);
        cv::cornerSubPix(gray, tmp, win_size, cv::Size(-1, -1), termcrit);
        points[1].push_back(tmp[0]);
        addRemovePt = false;
      }

      needToInit = false;
      if (debug_view_)
      {
        cv::imshow(window_name_, image);

        char c = (char)cv::waitKey(1);
        // if( c == 27 )
        //    break;
        switch (c)
        {
          case 'r':
            needToInit = true;
            break;
          case 'c':
            points[0].clear();
            points[1].clear();
            break;
          case 'n':
            nightMode = !nightMode;
            break;
        }
      }
      std::swap(points[1], points[0]);
      cv::swap(prevGray, gray);

      // Publish the image.
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_->publish(flows_msg);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(),
                   e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void initializePointsCb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                          std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    needToInit = true;
  }

  void deletePointsCb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    points[0].clear();
    points[1].clear();
  }

  void toggleNightModeCb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                         std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    nightMode = !nightMode;
  }

  void subscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Subscribing to image topic.");
    if (use_camera_info_)
      cam_sub_ = subscribeCamera("image", queue_size_, &LKFlowNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = subscribeImage("image", queue_size_, &LKFlowNodelet::imageCallback, this);
  }

  void unsubscribe() override
  {
    RCLCPP_DEBUG(get_logger(), "Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

public:
  explicit LKFlowNodelet(const rclcpp::NodeOptions& options) : Nodelet("lk_flow", options)
  {
    queue_size_ = declare_parameter("queue_size", 3);
    debug_view_ = declare_parameter("debug_view", false);
    if (debug_view_)
    {
      always_subscribe_ = true;
    }
    prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    window_name_ = "LK Demo";
    MAX_COUNT = 500;
    needToInit = true;
    num_detected_points_ = 0;
    nightMode = false;
    addRemovePt = false;

    use_camera_info_ = declare_parameter(
        "use_camera_info", false,
        paramDescriptor("Indicates that the camera_info topic should be subscribed to to get the default "
                        "input_frame_id. Otherwise the frame from the image message will be used."));
    quality_level_ = declare_parameter(
        "quality_level", 0.01,
        paramDescriptor("Parameter characterizing the minimal acceptance quality of image corners.", 0.001, 10.0));
    min_distance_ =
        declare_parameter("min_distance", 10,
                          paramDescriptor("Minimum possible Euclidean distance between the returned corners.", 1, 100));
    block_size_ = declare_parameter("block_size", 3,
                                    paramDescriptor("Size of an average block for computing a derivative covariation "
                                                    "matrix over each pixel neighborhood.",
                                                    1, 100));
    harris_k_ =
        declare_parameter("harris_k", 0.04, paramDescriptor("Free parameter of the Harris detector", 0.001, 10.0));
    on_set_parameters_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&LKFlowNodelet::parametersCallback, this, std::placeholders::_1));

    img_pub_ = advertiseImage("~/image", 1);
    msg_pub_ = advertise<opencv_apps::msg::FlowArrayStamped>("~/flows", 1);
    initialize_points_service_ = create_service<std_srvs::srv::Empty>(
        "~/initialize_points",
        std::bind(&LKFlowNodelet::initializePointsCb, this, std::placeholders::_1, std::placeholders::_2));
    delete_points_service_ = create_service<std_srvs::srv::Empty>(
        "~/delete_points",
        std::bind(&LKFlowNodelet::deletePointsCb, this, std::placeholders::_1, std::placeholders::_2));
    toggle_night_mode_service_ = create_service<std_srvs::srv::Empty>(
        "~/toggle_night_mode",
        std::bind(&LKFlowNodelet::toggleNightModeCb, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Hot keys: ");
    RCLCPP_INFO(get_logger(), "\tESC - quit the program");
    RCLCPP_INFO(get_logger(), "\tr - auto-initialize tracking");
    RCLCPP_INFO(get_logger(), "\tc - delete all the points");
    RCLCPP_INFO(get_logger(), "\tn - switch the \"night\" mode on/off");
    // NODELET_INFO("To add/remove a feature point click it");

    onInitPostProcess();
  }
};
bool LKFlowNodelet::need_config_update_ = false;
}  // namespace opencv_apps

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_apps::LKFlowNodelet)
