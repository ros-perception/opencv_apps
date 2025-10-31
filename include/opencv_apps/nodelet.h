/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Ryohei Ueda.
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

#ifndef OPENCV_APPS_NODELET_H_
#define OPENCV_APPS_NODELET_H_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <mutex>
#include <memory>

namespace opencv_apps
{

enum ConnectionStatus
{
  NOT_INITIALIZED,
  NOT_SUBSCRIBED,
  SUBSCRIBED
};

class Nodelet : public rclcpp::Node
{
public:
  explicit Nodelet(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node(node_name, options), subscribed_(false), ever_subscribed_(false),
      always_subscribe_(false), verbose_connection_(false), connection_status_(NOT_INITIALIZED)
  {
  }

  virtual ~Nodelet() = default;

protected:
  virtual void onInit();
  virtual void onInitPostProcess();
  virtual void subscribe() = 0;
  virtual void unsubscribe() = 0;

  template <class T>
  typename rclcpp::Publisher<T>::SharedPtr advertise(const std::string& topic, int queue_size)
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    auto pub = this->create_publisher<T>(topic, queue_size);
    // In ROS2, we don't have simple connection callbacks, so we'll just subscribe immediately if always_subscribe_ is true
    return pub;
  }

  std::shared_ptr<image_transport::Publisher> advertiseImage(const std::string& topic, int queue_size)
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    auto pub = std::make_shared<image_transport::Publisher>(
      image_transport::create_publisher(this, topic));
    image_publishers_.push_back(pub);
    return pub;
  }

  std::shared_ptr<image_transport::CameraPublisher> advertiseCamera(const std::string& topic, int queue_size)
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    auto pub = std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, topic));
    camera_publishers_.push_back(pub);
    return pub;
  }

  std::mutex connection_mutex_;
  std::vector<std::shared_ptr<image_transport::Publisher>> image_publishers_;
  std::vector<std::shared_ptr<image_transport::CameraPublisher>> camera_publishers_;

  bool subscribed_;
  bool ever_subscribed_;
  bool always_subscribe_;
  bool verbose_connection_;
  ConnectionStatus connection_status_;

private:
};

}  // namespace opencv_apps

#endif
