// -*- mode: c++ -*-
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

#include "opencv_apps/nodelet.h"

namespace opencv_apps
{
Nodelet::Nodelet(const std::string& node_name, const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options), ever_subscribed_(false), connection_status_(NOT_INITIALIZED)
{
  always_subscribe_ = declare_parameter("always_subscribe", false);
  verbose_connection_ = declare_parameter("verbose_connection", false);
  latch_ = declare_parameter("latch", false);
#if __has_include(<image_transport/node_interfaces.hpp>)
  it_ = std::make_shared<image_transport::ImageTransport>(*this);
#else
  // shared_from_this() is not available in constructor, and ImageTransport should not own this node
  it_ = std::make_shared<image_transport::ImageTransport>(rclcpp::Node::SharedPtr(this, [](rclcpp::Node*) {}));
#endif
  // timer to warn when no connection in a few seconds
  timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
    timer_->cancel();
    warnNeverSubscribedCallback();
  });
}

void Nodelet::onInitPostProcess()
{
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    connection_status_ = NOT_SUBSCRIBED;
  }
  if (always_subscribe_)
  {
    subscribe();
  }
  else
  {
    // subscribers may be already matched
    connectionCallback();
  }
}

void Nodelet::warnNeverSubscribedCallback()
{
  if (!ever_subscribed_ && !always_subscribe_)
  {
    RCLCPP_WARN(get_logger(), "'%s' subscribes topics only with child subscribers.", get_name());
  }
}

rclcpp::QoS Nodelet::qos(int queue_size) const
{
  rclcpp::QoS qos(queue_size);
  if (latch_)
  {
    qos.transient_local();
  }
  return qos;
}

rclcpp::PublisherOptions Nodelet::publisherOptions()
{
  rclcpp::PublisherOptions options;
  options.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo&) { connectionCallback(); };
  return options;
}

image_transport::Publisher Nodelet::advertiseImage(const std::string& topic, int queue_size)
{
  std::lock_guard<std::mutex> lock(connection_mutex_);
#if __has_include(<image_transport/node_interfaces.hpp>)
  auto pub = image_transport::create_publisher(*this, topic, qos(queue_size), publisherOptions());
#else
  auto pub = image_transport::create_publisher(this, topic, qos(queue_size).get_rmw_qos_profile(), publisherOptions());
#endif
  image_publishers_.push_back(pub);
  return pub;
}

image_transport::CameraPublisher Nodelet::advertiseCamera(const std::string& topic, int queue_size)
{
  std::lock_guard<std::mutex> lock(connection_mutex_);
#if __has_include(<image_transport/node_interfaces.hpp>)
  auto pub = image_transport::create_camera_publisher(*this, topic, qos(queue_size), publisherOptions());
#else
  auto pub =
      image_transport::create_camera_publisher(this, topic, qos(queue_size).get_rmw_qos_profile(), publisherOptions());
#endif
  camera_publishers_.push_back(pub);
  return pub;
}

void Nodelet::connectionCallback()
{
  if (verbose_connection_)
  {
    RCLCPP_INFO(get_logger(), "New connection or disconnection is detected");
  }
  if (always_subscribe_)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(connection_mutex_);
  if (connection_status_ == NOT_INITIALIZED)
  {
    return;
  }
  size_t num_subscribers = 0;
  for (const auto& pub : publishers_)
  {
    num_subscribers += pub->get_subscription_count();
  }
  for (const auto& pub : image_publishers_)
  {
    num_subscribers += pub.getNumSubscribers();
  }
  for (const auto& pub : camera_publishers_)
  {
    num_subscribers += pub.getNumSubscribers();
  }
  if (num_subscribers > 0)
  {
    ever_subscribed_ = true;
    if (connection_status_ != SUBSCRIBED)
    {
      if (verbose_connection_)
      {
        RCLCPP_INFO(get_logger(), "Subscribe input topics");
      }
      subscribe();
      connection_status_ = SUBSCRIBED;
    }
  }
  else if (connection_status_ == SUBSCRIBED)
  {
    if (verbose_connection_)
    {
      RCLCPP_INFO(get_logger(), "Unsubscribe input topics");
    }
    unsubscribe();
    connection_status_ = NOT_SUBSCRIBED;
  }
}

rcl_interfaces::msg::ParameterDescriptor paramDescriptor(const std::string& description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor paramDescriptor(const std::string& description, int from_value, int to_value)
{
  auto descriptor = paramDescriptor(description);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from_value;
  range.to_value = to_value;
  descriptor.integer_range.push_back(range);
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor paramDescriptor(const std::string& description, double from_value,
                                                         double to_value)
{
  auto descriptor = paramDescriptor(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from_value;
  range.to_value = to_value;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}
}  // namespace opencv_apps
