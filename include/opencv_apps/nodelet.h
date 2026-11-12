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
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace opencv_apps
{
/** @brief
 * Enum to represent connection status.
 */
enum ConnectionStatus
{
  NOT_INITIALIZED,
  NOT_SUBSCRIBED,
  SUBSCRIBED
};

/** @brief
 * Node to automatically subscribe/unsubscribe
 * topics according to subscription of advertised topics.
 *
 * It's important not to subscribe topic if no output is required.
 *
 * In order to watch advertised topics, need to use advertise template method.
 * And create subscribers in subscribe() and shutdown them in unsubscribed().
 *
 */
class Nodelet : public rclcpp::Node
{
public:
  Nodelet(const std::string& node_name, const rclcpp::NodeOptions& options);

protected:
  /** @brief
   * Post processing of initialization of nodelet.
   * Subclass should call this method at the end of its constructor
   */
  void onInitPostProcess();

  /** @brief
   * callback function which is called when subscribers of advertised topics are changed
   */
  void connectionCallback();

  /** @brief
   * callback function which is called when walltimer
   * duration run out.
   */
  void warnNeverSubscribedCallback();

  /** @brief
   * This method is called when publisher is subscribed by other
   * nodes.
   * Set up subscribers in this method.
   */
  virtual void subscribe() = 0;

  /** @brief
   * This method is called when publisher is unsubscribed by other
   * nodes.
   * Shut down subscribers in this method.
   */
  virtual void unsubscribe() = 0;

  /** @brief
   * Advertise a topic and watch the publisher.
   *
   * @param topic topic name to advertise.
   * @param queue_size queue size for publisher.
   * @return Publisher for the advertised topic.
   */
  template <class T>
  typename rclcpp::Publisher<T>::SharedPtr advertise(const std::string& topic, int queue_size)
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    auto pub = create_publisher<T>(topic, qos(queue_size), publisherOptions());
    publishers_.push_back(pub);
    return pub;
  }

  /** @brief
   * Advertise an image topic and watch the publisher.
   *
   * @param topic topic name to advertise.
   * @param queue_size queue size for publisher.
   * @return Publisher for the advertised topic.
   */
  image_transport::Publisher advertiseImage(const std::string& topic, int queue_size);

  /** @brief
   * Advertise an image topic camera info topic and watch the publisher.
   *
   * @param topic topic name to advertise.
   * @param queue_size queue size for publisher.
   * @return Publisher for the advertised topic.
   */
  image_transport::CameraPublisher advertiseCamera(const std::string& topic, int queue_size);

  /** @brief
   * Subscribe an image topic with subscriberQos(), and the transport of `~image_transport` parameter.
   *
   * @param topic topic name to subscribe.
   * @param queue_size queue size for subscriber.
   * @param fp member function to call with the image.
   * @param obj object to call fp.
   * @return Subscriber for the topic.
   */
  template <class F, class T>
  image_transport::Subscriber subscribeImage(const std::string& topic, int queue_size, F fp, T* obj)
  {
    auto callback = std::bind(fp, obj, std::placeholders::_1);
#if __has_include(<image_transport/node_interfaces.hpp>)
    return image_transport::create_subscription(
        *this, topic, callback, image_transport::TransportHints(*this).getTransport(), subscriberQos(queue_size));
#else
    return image_transport::create_subscription(this, topic, callback,
                                                image_transport::TransportHints(this).getTransport(),
                                                subscriberQos(queue_size).get_rmw_qos_profile());
#endif
  }

  /** @brief
   * Subscribe an image topic and its camera info topic with subscriberQos(), and the transport of
   * `~image_transport` parameter.
   *
   * @param topic topic name to subscribe.
   * @param queue_size queue size for subscriber.
   * @param fp member function to call with the image and the camera info.
   * @param obj object to call fp.
   * @return CameraSubscriber for the topics.
   */
  template <class F, class T>
  image_transport::CameraSubscriber subscribeCamera(const std::string& topic, int queue_size, F fp, T* obj)
  {
    auto callback = std::bind(fp, obj, std::placeholders::_1, std::placeholders::_2);
#if __has_include(<image_transport/node_interfaces.hpp>)
    return image_transport::create_camera_subscription(
        *this, topic, callback, image_transport::TransportHints(*this).getTransport(), subscriberQos(queue_size));
#else
    return image_transport::create_camera_subscription(this, topic, callback,
                                                       image_transport::TransportHints(this).getTransport(),
                                                       subscriberQos(queue_size).get_rmw_qos_profile());
#endif
  }

  /** @brief
   * QoS of advertised topics, transient local if `~latch` parameter is true.
   */
  rclcpp::QoS qos(int queue_size) const;

  /** @brief
   * QoS of subscribed topics, sensor data (best effort) to subscribe both reliable and best effort publishers, or
   * reliable with `~reliable_subscription` parameter, not to lose large messages, e.g. images through a network.
   */
  rclcpp::QoS subscriberQos(int queue_size) const;

  /** @brief
   * PublisherOptions to call connectionCallback() when subscribers are changed.
   */
  rclcpp::PublisherOptions publisherOptions();

  /** @brief
   * mutex to call subscribe() and unsubscribe() in
   * critical section.
   */
  std::mutex connection_mutex_;

  /** @brief
   * List of watching publishers
   */
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;

  /** @brief
   * List of watching image publishers
   */
  std::vector<image_transport::Publisher> image_publishers_;

  /** @brief
   * List of watching camera publishers
   */
  std::vector<image_transport::CameraPublisher> camera_publishers_;

  /** @brief
   * WallTimer instance for warning about no connection.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /** @brief
   * A flag to check if the node has been ever subscribed
   * or not.
   */
  bool ever_subscribed_;

  /** @brief
   * A flag to disable watching mechanism and always subscribe input
   * topics. It can be specified via ~always_subscribe parameter.
   */
  bool always_subscribe_;

  /** @brief
   * Status of connection
   */
  ConnectionStatus connection_status_;

  /** @brief
   * true if `~verbose_connection` parameter is true.
   */
  bool verbose_connection_;

  /** @brief
   * true if `~latch` parameter is true.
   */
  bool latch_;

  /** @brief
   * true if `~reliable_subscription` parameter is true, see subscriberQos().
   */
  bool reliable_subscription_;
};

/** @brief
 * ParameterDescriptor with description, and its range
 */
rcl_interfaces::msg::ParameterDescriptor paramDescriptor(const std::string& description);
rcl_interfaces::msg::ParameterDescriptor paramDescriptor(const std::string& description, int from_value, int to_value);
rcl_interfaces::msg::ParameterDescriptor paramDescriptor(const std::string& description, double from_value,
                                                         double to_value);
}  // namespace opencv_apps

#endif  // OPENCV_APPS_NODELET_H_
