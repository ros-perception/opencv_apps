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

// ROS1 only: the subset of the rclcpp API that the opencv_apps nodes use,
// implemented on top of roscpp/nodelet, so that the nodes written for ROS2
// also build with ROS1. ros1/include is only in the include path of the ROS1
// build, and its rclcpp/rclcpp.hpp, <pkg>/msg/foo.hpp, ... include this file.
//
// - rclcpp::Node is a class on top of the (private) NodeHandle of a nodelet;
//   RCLCPP_COMPONENTS_REGISTER_NODE(Foo) exports a nodelet that constructs
//   Foo in its onInit(), as type "Foo" (see nodelet_plugins.xml).
// - Parameters are read from the parameter server, and the parameters of a
//   node with add_on_set_parameters_callback() are served through the
//   dynamic_reconfigure protocol, so rqt_reconfigure / dynparam keep working.
// - Topic names starting with "~/" are private names.
// - <pkg>::msg::Foo and <pkg>::srv::Foo are ROS1 <pkg>::Foo, and
//   Foo::SharedPtr / Foo::ConstSharedPtr are Foo::Ptr / Foo::ConstPtr.
// - Within namespace opencv_apps, image_transport:: and message_filters::
//   also have the ROS2 style functions, e.g. Subscriber::subscribe(node, ...).

#ifndef OPENCV_APPS_ROS1_COMPAT_H_
#define OPENCV_APPS_ROS1_COMPAT_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <class_loader/class_loader.hpp>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// for compilers without __has_include (gcc < 5), nothing is there
#ifndef __has_include
#define __has_include(x) 0
#endif

// <pkg>::msg::Foo -> <pkg>::Foo
namespace geometry_msgs
{
namespace msg = ::geometry_msgs;
}
namespace sensor_msgs
{
namespace msg = ::sensor_msgs;
}
namespace std_msgs
{
namespace msg = ::std_msgs;
}
namespace std_srvs
{
namespace srv = ::std_srvs;
}
namespace opencv_apps
{
namespace msg = ::opencv_apps;
namespace srv = ::opencv_apps;
}  // namespace opencv_apps

enum rcl_clock_type_t
{
  RCL_CLOCK_UNINITIALIZED = 0,
  RCL_ROS_TIME,
  RCL_SYSTEM_TIME,
  RCL_STEADY_TIME
};

struct rmw_qos_profile_t
{
  size_t depth;
  bool transient_local;
};

namespace rcl_interfaces
{
namespace msg
{
struct IntegerRange
{
  IntegerRange() : from_value(0), to_value(0), step(0)
  {
  }
  int64_t from_value;
  int64_t to_value;
  uint64_t step;
};
struct FloatingPointRange
{
  FloatingPointRange() : from_value(0), to_value(0), step(0)
  {
  }
  double from_value;
  double to_value;
  double step;
};
struct ParameterDescriptor
{
  ParameterDescriptor() : read_only(false)
  {
  }
  std::string name;
  std::string description;
  bool read_only;
  std::vector<FloatingPointRange> floating_point_range;
  std::vector<IntegerRange> integer_range;
};
struct SetParametersResult
{
  SetParametersResult() : successful(false)
  {
  }
  bool successful;
  std::string reason;
};
}  // namespace msg
}  // namespace rcl_interfaces

namespace rclcpp
{
class Node;

/** @brief
 * Carries the NodeHandles of the nodelet to the constructor of the node.
 */
class NodeOptions
{
public:
  NodeOptions()
  {
  }
  NodeOptions(const std::string& name, const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : name_(name), nh_(nh), pnh_(pnh)
  {
  }
  std::string name_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};

class QoS
{
public:
  QoS(size_t depth) : depth_(depth), transient_local_(false)  // NOLINT(google-explicit-constructor)
  {
  }
  QoS& transient_local()
  {
    transient_local_ = true;
    return *this;
  }
  QoS& keep_last(size_t depth)
  {
    depth_ = depth;
    return *this;
  }
  size_t depth() const
  {
    return depth_;
  }
  bool is_transient_local() const
  {
    return transient_local_;
  }
  rmw_qos_profile_t get_rmw_qos_profile() const
  {
    rmw_qos_profile_t profile;
    profile.depth = depth_;
    profile.transient_local = transient_local_;
    return profile;
  }

private:
  size_t depth_;
  bool transient_local_;
};

// ROS1 has no best effort, only the depth of rclcpp::SensorDataQoS
class SensorDataQoS : public QoS
{
public:
  SensorDataQoS() : QoS(5)
  {
  }
};

class Time : public ros::Time
{
public:
  Time()
  {
  }
  Time(int32_t sec, uint32_t nsec, rcl_clock_type_t /* clock_type */ = RCL_SYSTEM_TIME) : ros::Time(sec, nsec)
  {
  }
  Time(const ros::Time& t) : ros::Time(t)  // NOLINT(google-explicit-constructor)
  {
  }
  double seconds() const
  {
    return toSec();
  }
  int64_t nanoseconds() const
  {
    return toNSec();
  }
};

class Clock
{
public:
  typedef std::shared_ptr<Clock> Ptr;
};

typedef std::string Logger;
inline Logger get_logger(const std::string& name)
{
  return name;
}

enum ParameterType
{
  PARAMETER_NOT_SET = 0,
  PARAMETER_BOOL,
  PARAMETER_INTEGER,
  PARAMETER_DOUBLE,
  PARAMETER_STRING,
  PARAMETER_DOUBLE_ARRAY
};

class Parameter
{
public:
  Parameter() : type_(PARAMETER_NOT_SET), bool_(false), int_(0), double_(0)
  {
  }
  Parameter(const std::string& name, bool v) : name_(name), type_(PARAMETER_BOOL), bool_(v), int_(0), double_(0)
  {
  }
  Parameter(const std::string& name, int v) : name_(name), type_(PARAMETER_INTEGER), bool_(false), int_(v), double_(0)
  {
  }
  Parameter(const std::string& name, int64_t v)
    : name_(name), type_(PARAMETER_INTEGER), bool_(false), int_(v), double_(0)
  {
  }
  Parameter(const std::string& name, double v) : name_(name), type_(PARAMETER_DOUBLE), bool_(false), int_(0), double_(v)
  {
  }
  Parameter(const std::string& name, const std::string& v)
    : name_(name), type_(PARAMETER_STRING), bool_(false), int_(0), double_(0), string_(v)
  {
  }
  Parameter(const std::string& name, const char* v)
    : name_(name), type_(PARAMETER_STRING), bool_(false), int_(0), double_(0), string_(v)
  {
  }
  Parameter(const std::string& name, const std::vector<double>& v)
    : name_(name), type_(PARAMETER_DOUBLE_ARRAY), bool_(false), int_(0), double_(0), double_array_(v)
  {
  }
  Parameter(const std::string& name, const std::vector<float>& v)
    : name_(name), type_(PARAMETER_DOUBLE_ARRAY), bool_(false), int_(0), double_(0), double_array_(v.begin(), v.end())
  {
  }
  const std::string& get_name() const
  {
    return name_;
  }
  ParameterType get_type() const
  {
    return type_;
  }
  bool as_bool() const
  {
    return bool_;
  }
  int64_t as_int() const
  {
    return int_;
  }
  double as_double() const
  {
    return double_;
  }
  const std::string& as_string() const
  {
    return string_;
  }
  const std::vector<double>& as_double_array() const
  {
    return double_array_;
  }

private:
  std::string name_;
  ParameterType type_;
  bool bool_;
  int64_t int_;
  double double_;
  std::string string_;
  std::vector<double> double_array_;
};

namespace node_interfaces
{
class OnSetParametersCallbackHandle
{
public:
  typedef std::shared_ptr<OnSetParametersCallbackHandle> Ptr;
  typedef std::function<rcl_interfaces::msg::SetParametersResult(const std::vector<rclcpp::Parameter>&)>
      OnParametersSetCallbackType;
  OnParametersSetCallbackType callback;
};
}  // namespace node_interfaces

struct MatchedInfo
{
};

/** @brief
 * matched_callback is called when a subscriber connects or disconnects
 */
struct PublisherOptions
{
  struct EventCallbacks
  {
    std::function<void(MatchedInfo&)> matched_callback;
  } event_callbacks;

  // callbacks for ros::NodeHandle::advertise() and image_transport
  void connectionCallback() const
  {
    if (event_callbacks.matched_callback)
    {
      MatchedInfo info;
      event_callbacks.matched_callback(info);
    }
  }
  ros::SubscriberStatusCallback subscriberStatusCallback() const
  {
    PublisherOptions self = *this;
    return [self](const ros::SingleSubscriberPublisher&) { self.connectionCallback(); };
  }
  image_transport::SubscriberStatusCallback imageSubscriberStatusCallback() const
  {
    PublisherOptions self = *this;
    return [self](const image_transport::SingleSubscriberPublisher&) { self.connectionCallback(); };
  }
};

class PublisherBase
{
public:
  typedef std::shared_ptr<PublisherBase> Ptr;
  explicit PublisherBase(const ros::Publisher& pub) : pub_(pub)
  {
  }
  virtual ~PublisherBase()
  {
  }
  size_t get_subscription_count() const
  {
    return pub_.getNumSubscribers();
  }

protected:
  ros::Publisher pub_;
};

template <class T>
class Publisher : public PublisherBase
{
public:
  typedef std::shared_ptr<Publisher<T> > Ptr;
  explicit Publisher(const ros::Publisher& pub) : PublisherBase(pub)
  {
  }
  void publish(const T& msg) const
  {
    pub_.publish(msg);
  }
  // takes the ownership as ROS2, and publishes it without copy to the subscribers in the same nodelet manager
  void publish(std::unique_ptr<T> msg) const
  {
    pub_.publish(boost::shared_ptr<T>(msg.release()));
  }
};

template <class T>
class Subscription
{
public:
  typedef std::shared_ptr<Subscription<T> > Ptr;
  explicit Subscription(const ros::Subscriber& sub) : sub_(sub)
  {
  }

private:
  ros::Subscriber sub_;
};

template <class T>
class Service
{
public:
  typedef std::shared_ptr<Service<T> > Ptr;
  explicit Service(const ros::ServiceServer& srv) : srv_(srv)
  {
  }

private:
  ros::ServiceServer srv_;
};

class TimerBase
{
public:
  typedef std::shared_ptr<TimerBase> Ptr;
  explicit TimerBase(const ros::WallTimer& timer) : timer_(timer)
  {
  }
  void cancel()
  {
    timer_.stop();
  }

private:
  ros::WallTimer timer_;
};

/** @brief
 * rclcpp::Node like API on top of the NodeHandles of a nodelet.
 */
class Node
{
public:
  typedef std::shared_ptr<Node> Ptr;

  Node(const std::string& /* node_name */, const NodeOptions& options)
    : name_(options.name_), nh_(options.nh_), pnh_(options.pnh_)
  {
  }
  virtual ~Node()
  {
  }

  template <class T>
  T declare_parameter(
      const std::string& name, const T& default_value,
      const rcl_interfaces::msg::ParameterDescriptor& descriptor = rcl_interfaces::msg::ParameterDescriptor())
  {
    T value;
    pnh_.param(name, value, default_value);
    value = clampToRange(descriptor, value);
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    parameters_[name] = rclcpp::Parameter(name, value);
    descriptors_[name] = descriptor;
    parameter_names_.push_back(name);
    return value;
  }

  std::string declare_parameter(
      const std::string& name, const char* default_value,
      const rcl_interfaces::msg::ParameterDescriptor& descriptor = rcl_interfaces::msg::ParameterDescriptor())
  {
    return declare_parameter(name, std::string(default_value), descriptor);
  }

  bool has_parameter(const std::string& name) const
  {
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    return parameters_.count(name) > 0;
  }

  rclcpp::Parameter get_parameter(const std::string& name) const
  {
    if (name == "use_sim_time")
    {
      return rclcpp::Parameter(name, ros::Time::isSimTime());
    }
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    std::map<std::string, rclcpp::Parameter>::const_iterator it = parameters_.find(name);
    if (it == parameters_.end())
    {
      throw std::runtime_error("parameter '" + name + "' is not declared");
    }
    return it->second;
  }

  rcl_interfaces::msg::SetParametersResult set_parameter(const rclcpp::Parameter& parameter)
  {
    rcl_interfaces::msg::SetParametersResult result = setParameters(std::vector<rclcpp::Parameter>(1, parameter));
    if (result.successful)
    {
      publishParameterUpdates();
    }
    return result;
  }

  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(const std::vector<rclcpp::Parameter>& parameters)
  {
    std::vector<rcl_interfaces::msg::SetParametersResult> results;
    for (size_t i = 0; i < parameters.size(); i++)
    {
      results.push_back(set_parameter(parameters[i]));
    }
    return results;
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::Ptr add_on_set_parameters_callback(
      rclcpp::node_interfaces::OnSetParametersCallbackHandle::OnParametersSetCallbackType callback)
  {
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::Ptr handle =
        std::make_shared<rclcpp::node_interfaces::OnSetParametersCallbackHandle>();
    handle->callback = callback;
    on_set_parameters_callbacks_.push_back(handle);
    startReconfigureServer();
    return handle;
  }

  template <class T>
  typename rclcpp::Publisher<T>::Ptr create_publisher(const std::string& topic, const rclcpp::QoS& qos,
                                                      const rclcpp::PublisherOptions& options = PublisherOptions())
  {
    ros::Publisher pub =
        ros1NodeHandle(topic).advertise<T>(ros1TopicName(topic), qos.depth(), options.subscriberStatusCallback(),
                                           options.subscriberStatusCallback(), ros::VoidConstPtr(),
                                           qos.is_transient_local());
    return std::make_shared<rclcpp::Publisher<T> >(pub);
  }

  template <class T, class C>
  typename rclcpp::Subscription<T>::Ptr create_subscription(const std::string& topic, const rclcpp::QoS& qos, C callback)
  {
    ros::Subscriber sub =
        ros1NodeHandle(topic).subscribe<T>(ros1TopicName(topic), qos.depth(), ros1SubscriptionCallback<T>(callback, 0));
    return std::make_shared<rclcpp::Subscription<T> >(sub);
  }

  template <class T, class C>
  typename rclcpp::Service<T>::Ptr create_service(const std::string& name, C callback)
  {
    // the request and the response are owned by roscpp
    boost::function<bool(typename T::Request&, typename T::Response&)> ros1_callback =
        [callback](typename T::Request& req, typename T::Response& res) {
          callback(std::shared_ptr<typename T::Request>(&req, [](typename T::Request*) {}),
                   std::shared_ptr<typename T::Response>(&res, [](typename T::Response*) {}));
          return true;
        };
    ros::ServiceServer srv = ros1NodeHandle(name).advertiseService<typename T::Request, typename T::Response>(
        ros1TopicName(name), ros1_callback);
    return std::make_shared<rclcpp::Service<T> >(srv);
  }

  template <class Rep, class Period>
  rclcpp::TimerBase::Ptr create_wall_timer(std::chrono::duration<Rep, Period> period, std::function<void()> callback)
  {
    ros::WallTimer timer = nh_.createWallTimer(ros::WallDuration(std::chrono::duration<double>(period).count()),
                                               [callback](const ros::WallTimerEvent&) { callback(); });
    return std::make_shared<rclcpp::TimerBase>(timer);
  }

  rclcpp::Logger get_logger() const
  {
    return name_;
  }

  const char* get_name() const
  {
    return name_.c_str();
  }

  const char* get_fully_qualified_name() const
  {
    return name_.c_str();
  }

  rclcpp::Clock::Ptr get_clock() const
  {
    return rclcpp::Clock::Ptr();
  }

  rclcpp::Time now() const
  {
    return ros::Time::now();
  }

  /** @brief
   * NodeHandle for a topic: the private one for "~/foo", otherwise the one of the namespace
   */
  ros::NodeHandle& ros1NodeHandle(const std::string& topic)
  {
    return (topic.compare(0, 2, "~/") == 0) ? pnh_ : nh_;
  }
  static std::string ros1TopicName(const std::string& topic)
  {
    return (topic.compare(0, 2, "~/") == 0) ? topic.substr(2) : topic;
  }

private:
  template <class T>
  using Ros1Callback = boost::function<void(const typename T::ConstPtr&)>;
  // callback of create_subscription() takes ConstSharedPtr, or the message
  template <class T, class C>
  static auto ros1SubscriptionCallback(C callback, int) -> decltype(callback(typename T::ConstPtr()), Ros1Callback<T>())
  {
    return callback;
  }
  template <class T, class C>
  static Ros1Callback<T> ros1SubscriptionCallback(C callback, long)
  {
    return [callback](const typename T::ConstPtr& msg) { callback(*msg); };
  }

  rcl_interfaces::msg::SetParametersResult setParameters(const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (size_t i = 0; i < on_set_parameters_callbacks_.size() && result.successful; i++)
    {
      result = on_set_parameters_callbacks_[i]->callback(parameters);
    }
    if (result.successful)
    {
      std::lock_guard<std::mutex> lock(parameters_mutex_);
      for (size_t i = 0; i < parameters.size(); i++)
      {
        parameters_[parameters[i].get_name()] = parameters[i];
      }
    }
    return result;
  }

  // dynamic_reconfigure server of the declared parameters
  void startReconfigureServer()
  {
    if (reconfigure_service_)
    {
      return;
    }
    dynamic_reconfigure::ConfigDescription description;
    dynamic_reconfigure::Group group;
    group.name = "Default";
    group.type = "";
    group.parent = 0;
    group.id = 0;
    {
      std::lock_guard<std::mutex> lock(parameters_mutex_);
      for (size_t i = 0; i < parameter_names_.size(); i++)
      {
        const rclcpp::Parameter& parameter = parameters_[parameter_names_[i]];
        const rcl_interfaces::msg::ParameterDescriptor& descriptor = descriptors_[parameter_names_[i]];
        dynamic_reconfigure::ParamDescription param;
        param.name = parameter.get_name();
        param.level = 0;
        param.description = descriptor.description;
        switch (parameter.get_type())
        {
          case rclcpp::PARAMETER_BOOL:
            param.type = "bool";
            appendValue(description.dflt, param.name, parameter.as_bool());
            appendValue(description.min, param.name, false);
            appendValue(description.max, param.name, true);
            break;
          case rclcpp::PARAMETER_INTEGER:
            param.type = "int";
            appendValue(description.dflt, param.name, static_cast<int>(parameter.as_int()));
            appendValue(description.min, param.name,
                        descriptor.integer_range.empty() ? INT_MIN :
                                                           static_cast<int>(descriptor.integer_range[0].from_value));
            appendValue(description.max, param.name,
                        descriptor.integer_range.empty() ? INT_MAX :
                                                           static_cast<int>(descriptor.integer_range[0].to_value));
            break;
          case rclcpp::PARAMETER_DOUBLE:
            param.type = "double";
            appendValue(description.dflt, param.name, parameter.as_double());
            appendValue(description.min, param.name,
                        descriptor.floating_point_range.empty() ? -std::numeric_limits<double>::infinity() :
                                                                  descriptor.floating_point_range[0].from_value);
            appendValue(description.max, param.name,
                        descriptor.floating_point_range.empty() ? std::numeric_limits<double>::infinity() :
                                                                  descriptor.floating_point_range[0].to_value);
            break;
          case rclcpp::PARAMETER_STRING:
            param.type = "str";
            appendValue(description.dflt, param.name, parameter.as_string());
            appendValue(description.min, param.name, std::string(""));
            appendValue(description.max, param.name, std::string(""));
            break;
          default:
            continue;
        }
        group.parameters.push_back(param);
      }
    }
    description.groups.push_back(group);
    dynamic_reconfigure::GroupState group_state;
    group_state.name = group.name;
    group_state.state = true;
    group_state.id = group.id;
    group_state.parent = group.parent;
    description.dflt.groups.push_back(group_state);
    description.min.groups.push_back(group_state);
    description.max.groups.push_back(group_state);

    descriptions_pub_ = pnh_.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
    descriptions_pub_.publish(description);
    updates_pub_ = pnh_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);
    publishParameterUpdates();
    reconfigure_service_ = pnh_.advertiseService("set_parameters", &Node::reconfigureCallback, this);
  }

  bool reconfigureCallback(dynamic_reconfigure::Reconfigure::Request& req,
                           dynamic_reconfigure::Reconfigure::Response& res)
  {
    std::vector<rclcpp::Parameter> parameters;
    for (size_t i = 0; i < req.config.bools.size(); i++)
    {
      parameters.push_back(rclcpp::Parameter(req.config.bools[i].name, static_cast<bool>(req.config.bools[i].value)));
    }
    for (size_t i = 0; i < req.config.ints.size(); i++)
    {
      const std::string& name = req.config.ints[i].name;
      parameters.push_back(
          rclcpp::Parameter(name, clampToRange(descriptor(name), static_cast<int>(req.config.ints[i].value))));
    }
    for (size_t i = 0; i < req.config.doubles.size(); i++)
    {
      const std::string& name = req.config.doubles[i].name;
      parameters.push_back(rclcpp::Parameter(name, clampToRange(descriptor(name), req.config.doubles[i].value)));
    }
    for (size_t i = 0; i < req.config.strs.size(); i++)
    {
      parameters.push_back(rclcpp::Parameter(req.config.strs[i].name, req.config.strs[i].value));
    }
    setParameters(parameters);
    res.config = parameterUpdates();
    updates_pub_.publish(res.config);
    return true;
  }

  rcl_interfaces::msg::ParameterDescriptor descriptor(const std::string& name)
  {
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    std::map<std::string, rcl_interfaces::msg::ParameterDescriptor>::const_iterator it = descriptors_.find(name);
    return (it == descriptors_.end()) ? rcl_interfaces::msg::ParameterDescriptor() : it->second;
  }

  // clamp to the range of the descriptor, as dynamic_reconfigure does, while ROS2 rejects the value
  template <class T>
  static T clampToRange(const rcl_interfaces::msg::ParameterDescriptor& /* descriptor */, const T& value)
  {
    return value;
  }
  static int clampToRange(const rcl_interfaces::msg::ParameterDescriptor& descriptor, int value)
  {
    if (descriptor.integer_range.empty())
    {
      return value;
    }
    // in int64_t, as the range may not fit in int
    int64_t from = std::max(descriptor.integer_range[0].from_value, static_cast<int64_t>(INT_MIN));
    int64_t to = std::min(descriptor.integer_range[0].to_value, static_cast<int64_t>(INT_MAX));
    return static_cast<int>(std::min(std::max(static_cast<int64_t>(value), from), to));
  }
  static double clampToRange(const rcl_interfaces::msg::ParameterDescriptor& descriptor, double value)
  {
    if (descriptor.floating_point_range.empty())
    {
      return value;
    }
    return std::min(std::max(value, descriptor.floating_point_range[0].from_value),
                    descriptor.floating_point_range[0].to_value);
  }

  // current values, which are also set to the parameter server as dynamic_reconfigure does
  dynamic_reconfigure::Config parameterUpdates()
  {
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    dynamic_reconfigure::Config config;
    for (size_t i = 0; i < parameter_names_.size(); i++)
    {
      const rclcpp::Parameter& parameter = parameters_[parameter_names_[i]];
      switch (parameter.get_type())
      {
        case rclcpp::PARAMETER_BOOL:
          appendValue(config, parameter.get_name(), parameter.as_bool());
          pnh_.setParam(parameter.get_name(), parameter.as_bool());
          break;
        case rclcpp::PARAMETER_INTEGER:
          appendValue(config, parameter.get_name(), static_cast<int>(parameter.as_int()));
          pnh_.setParam(parameter.get_name(), static_cast<int>(parameter.as_int()));
          break;
        case rclcpp::PARAMETER_DOUBLE:
          appendValue(config, parameter.get_name(), parameter.as_double());
          pnh_.setParam(parameter.get_name(), parameter.as_double());
          break;
        case rclcpp::PARAMETER_STRING:
          appendValue(config, parameter.get_name(), parameter.as_string());
          pnh_.setParam(parameter.get_name(), parameter.as_string());
          break;
        default:
          break;
      }
    }
    dynamic_reconfigure::GroupState group_state;
    group_state.name = "Default";
    group_state.state = true;
    group_state.id = 0;
    group_state.parent = 0;
    config.groups.push_back(group_state);
    return config;
  }

  void publishParameterUpdates()
  {
    if (updates_pub_)
    {
      updates_pub_.publish(parameterUpdates());
    }
  }

  static void appendValue(dynamic_reconfigure::Config& config, const std::string& name, bool value)
  {
    dynamic_reconfigure::BoolParameter p;
    p.name = name;
    p.value = value;
    config.bools.push_back(p);
  }
  static void appendValue(dynamic_reconfigure::Config& config, const std::string& name, int value)
  {
    dynamic_reconfigure::IntParameter p;
    p.name = name;
    p.value = value;
    config.ints.push_back(p);
  }
  static void appendValue(dynamic_reconfigure::Config& config, const std::string& name, double value)
  {
    dynamic_reconfigure::DoubleParameter p;
    p.name = name;
    p.value = value;
    config.doubles.push_back(p);
  }
  static void appendValue(dynamic_reconfigure::Config& config, const std::string& name, const std::string& value)
  {
    dynamic_reconfigure::StrParameter p;
    p.name = name;
    p.value = value;
    config.strs.push_back(p);
  }

  std::string name_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::map<std::string, rclcpp::Parameter> parameters_;
  std::map<std::string, rcl_interfaces::msg::ParameterDescriptor> descriptors_;
  std::vector<std::string> parameter_names_;
  std::vector<rclcpp::node_interfaces::OnSetParametersCallbackHandle::Ptr> on_set_parameters_callbacks_;
  mutable std::mutex parameters_mutex_;
  ros::Publisher descriptions_pub_;
  ros::Publisher updates_pub_;
  ros::ServiceServer reconfigure_service_;
};
}  // namespace rclcpp

// RCLCPP_* logging macros, the logger is the name of the nodelet as NODELET_* macros
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG_NAMED(logger, __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) ROS_INFO_NAMED(logger, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN_NAMED(logger, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR_NAMED(logger, __VA_ARGS__)
#define RCLCPP_FATAL(logger, ...) ROS_FATAL_NAMED(logger, __VA_ARGS__)
#define RCLCPP_DEBUG_STREAM(logger, args) ROS_DEBUG_STREAM_NAMED(logger, args)
#define RCLCPP_INFO_STREAM(logger, args) ROS_INFO_STREAM_NAMED(logger, args)
#define RCLCPP_WARN_STREAM(logger, args) ROS_WARN_STREAM_NAMED(logger, args)
#define RCLCPP_ERROR_STREAM(logger, args) ROS_ERROR_STREAM_NAMED(logger, args)
#define RCLCPP_FATAL_STREAM(logger, args) ROS_FATAL_STREAM_NAMED(logger, args)
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) ROS_WARN_THROTTLE_NAMED((ms) / 1000.0, logger, __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, ms, ...) ROS_ERROR_THROTTLE_NAMED((ms) / 1000.0, logger, __VA_ARGS__)

namespace opencv_apps
{
// image_transport:: with the ROS2 style functions, within namespace opencv_apps
namespace image_transport
{
using namespace ::image_transport;  // NOLINT(build/namespaces)

// reads `~image_transport` parameter of the node, as TransportHints(node) of ROS2
class TransportHints : public ::image_transport::TransportHints
{
public:
  explicit TransportHints(rclcpp::Node* node)
    : ::image_transport::TransportHints("raw", ros::TransportHints(), node->ros1NodeHandle("~/"))
  {
  }
};

class SubscriberFilter : public ::image_transport::SubscriberFilter
{
public:
  using ::image_transport::SubscriberFilter::subscribe;
  void subscribe(rclcpp::Node* node, const std::string& base_topic, const std::string& transport,
                 const rmw_qos_profile_t& qos)
  {
    ::image_transport::ImageTransport it(node->ros1NodeHandle(base_topic));
    subscribe(it, rclcpp::Node::ros1TopicName(base_topic), qos.depth, ::image_transport::TransportHints(transport));
  }
};

inline Publisher create_publisher(rclcpp::Node* node, const std::string& base_topic, const rmw_qos_profile_t& qos,
                                  const rclcpp::PublisherOptions& options = rclcpp::PublisherOptions())
{
  return ::image_transport::ImageTransport(node->ros1NodeHandle(base_topic))
      .advertise(rclcpp::Node::ros1TopicName(base_topic), qos.depth, options.imageSubscriberStatusCallback(),
                 options.imageSubscriberStatusCallback(), ros::VoidPtr(), qos.transient_local);
}

inline CameraPublisher create_camera_publisher(rclcpp::Node* node, const std::string& base_topic,
                                               const rmw_qos_profile_t& qos,
                                               const rclcpp::PublisherOptions& options = rclcpp::PublisherOptions())
{
  return ::image_transport::ImageTransport(node->ros1NodeHandle(base_topic))
      .advertiseCamera(rclcpp::Node::ros1TopicName(base_topic), qos.depth, options.imageSubscriberStatusCallback(),
                       options.imageSubscriberStatusCallback(), options.subscriberStatusCallback(),
                       options.subscriberStatusCallback(), ros::VoidPtr(), qos.transient_local);
}

template <class C>
Subscriber create_subscription(rclcpp::Node* node, const std::string& base_topic, C callback,
                               const std::string& transport, const rmw_qos_profile_t& qos)
{
  return ::image_transport::ImageTransport(node->ros1NodeHandle(base_topic))
      .subscribe(rclcpp::Node::ros1TopicName(base_topic), qos.depth,
                 boost::function<void(const sensor_msgs::ImageConstPtr&)>(callback), ros::VoidPtr(),
                 ::image_transport::TransportHints(transport));
}

template <class C>
CameraSubscriber create_camera_subscription(rclcpp::Node* node, const std::string& base_topic, C callback,
                                            const std::string& transport, const rmw_qos_profile_t& qos)
{
  return ::image_transport::ImageTransport(node->ros1NodeHandle(base_topic))
      .subscribeCamera(rclcpp::Node::ros1TopicName(base_topic), qos.depth, CameraSubscriber::Callback(callback),
                       ros::VoidPtr(), ::image_transport::TransportHints(transport));
}
}  // namespace image_transport

// message_filters:: with the ROS2 style functions, within namespace opencv_apps
namespace message_filters
{
using namespace ::message_filters;  // NOLINT(build/namespaces)

template <class M>
class Subscriber : public ::message_filters::Subscriber<M>
{
public:
  using ::message_filters::Subscriber<M>::subscribe;
  void subscribe(rclcpp::Node* node, const std::string& topic, const rmw_qos_profile_t& qos)
  {
    subscribe(node->ros1NodeHandle(topic), rclcpp::Node::ros1TopicName(topic), qos.depth);
  }
};
}  // namespace message_filters

/** @brief
 * Nodelet which constructs the node T in onInit()
 */
template <class T>
class NodeletWrapper : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    node_ = std::make_shared<T>(rclcpp::NodeOptions(getName(), getMTNodeHandle(), getMTPrivateNodeHandle()));
  }

private:
  std::shared_ptr<T> node_;
};

/** @brief
 * NodeletWrapper for the deprecated name of the nodelet, e.g. edge_detection/edge_detection
 */
template <class T>
class DeprecatedNodeletWrapper : public NodeletWrapper<T>
{
public:
  void onInit() override
  {
    ROS_WARN("DeprecationWarning: Nodelet %s is deprecated, and renamed to %s.", deprecatedName().c_str(),
             className().c_str());
    NodeletWrapper<T>::onInit();
  }
  static std::string& className()
  {
    static std::string name;
    return name;
  }
  static std::string& deprecatedName()
  {
    static std::string name;
    return name;
  }
};

// deprecated namespace of the nodelet class, e.g. edge_detection for opencv_apps::EdgeDetectionNodelet
inline std::string deprecatedNamespace(const std::string& class_name)
{
  if (class_name.find("ColorFilterNodelet") != std::string::npos)
  {
    return "color_filter";
  }
  std::string name = class_name.substr(class_name.rfind(':') + 1);
  name = name.substr(0, name.size() - std::string("Nodelet").size());
  static const char* irregular[][2] = { { "CamShift", "camshift" },
                                        { "FBackFlow", "fback_flow" },
                                        { "LKFlow", "lk_flow" } };
  for (size_t i = 0; i < sizeof(irregular) / sizeof(irregular[0]); i++)
  {
    if (name == irregular[i][0])
    {
      return irregular[i][1];
    }
  }
  std::string ns;
  for (size_t i = 0; i < name.size(); i++)
  {
    if (isupper(name[i]) && i > 0)
    {
      ns += '_';
    }
    ns += static_cast<char>(tolower(name[i]));
  }
  return ns;
}

/** @brief
 * Register the nodelet of the node T as "class_name", and also as "<deprecated namespace>::<class>"
 */
template <class T>
struct NodeletRegistrar
{
  explicit NodeletRegistrar(const std::string& class_name)
  {
#ifdef USE_PLUGINLIB_CLASS_LIST_MACROS_H
    namespace impl = class_loader::class_loader_private;
#else
    namespace impl = class_loader::impl;
#endif
    impl::registerPlugin<NodeletWrapper<T>, nodelet::Nodelet>(class_name, "nodelet::Nodelet");
    std::string deprecated_name = deprecatedNamespace(class_name) + "::" + class_name.substr(class_name.rfind(':') + 1);
    DeprecatedNodeletWrapper<T>::className() = class_name;
    DeprecatedNodeletWrapper<T>::deprecatedName() = deprecated_name;
    impl::registerPlugin<DeprecatedNodeletWrapper<T>, nodelet::Nodelet>(deprecated_name, "nodelet::Nodelet");
  }
};
}  // namespace opencv_apps

#define OPENCV_APPS_REGISTER_NODELET_CAT(x, y) x##y
#define OPENCV_APPS_REGISTER_NODELET(class_type, id)                                                                   \
  namespace                                                                                                            \
  {                                                                                                                    \
  const opencv_apps::NodeletRegistrar<class_type> OPENCV_APPS_REGISTER_NODELET_CAT(opencv_apps_nodelet_registrar_,     \
                                                                                   id)(#class_type);                   \
  }
// exports the node as a nodelet
#define RCLCPP_COMPONENTS_REGISTER_NODE(class_type) OPENCV_APPS_REGISTER_NODELET(class_type, __COUNTER__)

// Foo::SharedPtr -> Foo::Ptr, Foo::ConstSharedPtr -> Foo::ConstPtr
#define SharedPtr Ptr
#define ConstSharedPtr ConstPtr

#endif  // OPENCV_APPS_ROS1_COMPAT_H_
