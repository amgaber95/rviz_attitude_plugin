/*
 * RViz Attitude Display Plugin - Topic utilities (header-only)
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__TOPIC_UTILITIES_HPP_
#define RVIZ_ATTITUDE_PLUGIN__TOPIC_UTILITIES_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "rviz_attitude_plugin/supported_types.hpp"

namespace rviz_attitude_plugin
{

/**
 * @brief Header-only helper for discovering RViz-supported attitude topics.
 *
 * Queries the ROS2 graph via rclcpp, filtering topics so only those publishing
 * supported orientation message types (as defined in SupportedTypes) are
 * returned. Results map topic names to the first supported message type.
 */
class TopicDiscovery
{
public:
  TopicDiscovery() = default;
  ~TopicDiscovery() = default;

  inline std::map<std::string, std::string> discoverTopics(rclcpp::Node * node) const
  {
    std::map<std::string, std::string> supported;
    if (!node) {
      return supported;
    }

    const auto topics = node->get_topic_names_and_types();
    for (const auto & entry : topics) {
      const auto & topic_name = entry.first;
      const auto & types = entry.second;
      const std::string preferred = SupportedTypes::firstSupported(types);
      if (!preferred.empty()) {
        supported.emplace(topic_name, preferred);
      }
    }
    return supported;
  }
};

/**
 * @brief Type-erased subscriber that normalizes quaternion delivery.
 *
 * Creates message-specific subscriptions for supported message types and
 * invokes a unified callback that receives geometry_msgs::msg::Quaternion.
 */
class AttitudeSubscriber
{
public:
  using OrientationCallback = std::function<void(const geometry_msgs::msg::Quaternion &)>;

  AttitudeSubscriber() = default;
  ~AttitudeSubscriber() = default;

  inline bool start(rclcpp::Node * node,
                    const std::string & topic,
                    const std::string & type,
                    const OrientationCallback & callback)
  {
    stop();

    if (!node || topic.empty() || type.empty() || !SupportedTypes::isSupported(type)) {
      return false;
    }

    const rclcpp::QoS default_qos{rclcpp::KeepLast(10)};
    const rclcpp::SensorDataQoS sensor_qos{};

    if (type == SupportedTypes::Quaternion) {
      subscription_ = makeSubscription<geometry_msgs::msg::Quaternion>(
        node, topic, default_qos, callback);
    } else if (type == SupportedTypes::QuaternionStamped) {
      subscription_ = makeSubscription<geometry_msgs::msg::QuaternionStamped>(
        node, topic, default_qos, callback);
    } else if (type == SupportedTypes::Pose) {
      subscription_ = makeSubscription<geometry_msgs::msg::Pose>(
        node, topic, default_qos, callback);
    } else if (type == SupportedTypes::PoseStamped) {
      subscription_ = makeSubscription<geometry_msgs::msg::PoseStamped>(
        node, topic, default_qos, callback);
    } else if (type == SupportedTypes::PoseWithCovariance) {
      subscription_ = makeSubscription<geometry_msgs::msg::PoseWithCovariance>(
        node, topic, default_qos, callback);
    } else if (type == SupportedTypes::PoseWithCovarianceStamped) {
      subscription_ = makeSubscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        node, topic, default_qos, callback);
    } else if (type == SupportedTypes::Imu) {
      subscription_ = makeSubscription<sensor_msgs::msg::Imu>(
        node, topic, sensor_qos, callback);
    } else if (type == SupportedTypes::Odometry) {
      subscription_ = makeSubscription<nav_msgs::msg::Odometry>(
        node, topic, default_qos, callback);
    }

    return static_cast<bool>(subscription_);
  }

  inline void stop()
  {
    subscription_.reset();
  }

  inline bool active() const
  {
    return static_cast<bool>(subscription_);
  }

private:
  template<typename MsgT>
  rclcpp::SubscriptionBase::SharedPtr makeSubscription(
    rclcpp::Node * node,
    const std::string & topic,
    const rclcpp::QoS & qos,
    const OrientationCallback & callback) const
  {
    using MsgConstPtr = typename MsgT::ConstSharedPtr;
    auto sub = node->create_subscription<MsgT>(
      topic,
      qos,
      [callback](MsgConstPtr message)
      {
        if (callback) {
          callback(extract(*message));
        }
      });
    return std::static_pointer_cast<rclcpp::SubscriptionBase>(sub);
  }

  rclcpp::SubscriptionBase::SharedPtr subscription_;
};

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__TOPIC_UTILITIES_HPP_
