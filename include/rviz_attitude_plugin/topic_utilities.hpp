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
#include <rclcpp/subscription_base.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "rviz_attitude_plugin/supported_types.hpp"

namespace rviz_attitude_plugin
{

class TopicDiscovery
{
public:
  inline std::vector<std::pair<std::string, std::string>> list(rclcpp::Node * node) const
  {
    std::vector<std::pair<std::string, std::string>> result;
    if (!node) return result;

    std::map<std::string, std::vector<std::string>> topics = node->get_topic_names_and_types();

    for (const auto & kv : topics) {
      const auto & name = kv.first;
      const auto & types = kv.second;
      const std::string chosen = SupportedTypes::firstSupported(types);
      if (!chosen.empty()) {
        result.emplace_back(name, chosen);
      }
    }
    return result;
  }
};

class AttitudeSubscriber
{
public:
  using OrientationCallback = std::function<void(const geometry_msgs::msg::Quaternion &)>;

  inline void start(rclcpp::Node * node,
                    const std::string & topic,
                    const std::string & type,
                    const OrientationCallback & on_orientation)
  {
    stop();
    if (!node) return;
    if (!SupportedTypes::isSupported(type)) return;

    auto qos_default = rclcpp::QoS(10);
    auto qos_imu = rclcpp::SensorDataQoS();

    using rviz_attitude_plugin::extract;

    if (type == SupportedTypes::Quaternion) {
      sub_ = node->create_subscription<geometry_msgs::msg::Quaternion>(
        topic, qos_default,
        [on_orientation](geometry_msgs::msg::Quaternion::ConstSharedPtr m){ on_orientation(*m); });
    } else if (type == SupportedTypes::QuaternionStamped) {
      sub_ = node->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        topic, qos_default,
        [on_orientation](geometry_msgs::msg::QuaternionStamped::ConstSharedPtr m){ on_orientation(extract(*m)); });
    } else if (type == SupportedTypes::Pose) {
      sub_ = node->create_subscription<geometry_msgs::msg::Pose>(
        topic, qos_default,
        [on_orientation](geometry_msgs::msg::Pose::ConstSharedPtr m){ on_orientation(extract(*m)); });
    } else if (type == SupportedTypes::PoseStamped) {
      sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, qos_default,
        [on_orientation](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){ on_orientation(extract(*m)); });
    } else if (type == SupportedTypes::PoseWithCovariance) {
      sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovariance>(
        topic, qos_default,
        [on_orientation](geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr m){ on_orientation(extract(*m)); });
    } else if (type == SupportedTypes::PoseWithCovarianceStamped) {
      sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic, qos_default,
        [on_orientation](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){ on_orientation(extract(*m)); });
    } else if (type == SupportedTypes::Imu) {
      sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
        topic, qos_imu,
        [on_orientation](sensor_msgs::msg::Imu::ConstSharedPtr m){ on_orientation(extract(*m)); });
    } else if (type == SupportedTypes::Odometry) {
      sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        topic, qos_default,
        [on_orientation](nav_msgs::msg::Odometry::ConstSharedPtr m){ on_orientation(extract(*m)); });
    }
  }

  /**
   * @brief Stop the current subscription.
   */
  inline void stop()
  {
    sub_.reset();
  }

private:
  rclcpp::SubscriptionBase::SharedPtr sub_;
};

class AttitudeTopicManager
{
public:
  using OrientationCallback = std::function<void(const geometry_msgs::msg::Quaternion &)>;
  using TopicList = std::vector<std::pair<std::string, std::string>>;

  /**
   * @brief Refresh the list of available topics with supported types.
   * @param node ROS2 node to query
   * @return Vector of (topic_name, type_string) pairs
   */
  inline TopicList refreshTopics(rclcpp::Node * node)
  {
    if (!node) {
      cached_topics_.clear();
      return {};
    }

    cached_topics_ = topic_discovery_.list(node);
    return cached_topics_;
  }

  /**
   * @brief Get the cached list of topics (avoids repeated queries).
   * @return Vector of (topic_name, type_string) pairs
   */
  inline const TopicList & getCachedTopics() const
  {
    return cached_topics_;
  }

  /**
   * @brief Resolve the message type for a specific topic.
   * @param node ROS2 node to query
   * @param topic Topic name to resolve
   * @return First supported type for the topic, or empty string if not found
   */
  inline std::string resolveType(rclcpp::Node * node, const std::string & topic)
  {
    if (!node || topic.empty()) return {};

    auto topics = node->get_topic_names_and_types();
    auto it = topics.find(topic);
    if (it == topics.end()) return {};

    return SupportedTypes::firstSupported(it->second);
  }

  inline bool subscribe(rclcpp::Node * node,
                        const std::string & topic,
                        const std::string & type,
                        const OrientationCallback & callback)
  {
    if (!node || topic.empty() || type.empty()) return false;
    if (!SupportedTypes::isSupported(type)) return false;

    // Stop any existing subscription
    unsubscribe();

    // Start new subscription
    attitude_subscriber_.start(node, topic, type, callback);

    // Update state
    active_topic_ = topic;
    active_type_ = type;

    return true;
  }

  /**
   * @brief Unsubscribe from the current topic.
   */
  inline void unsubscribe()
  {
    attitude_subscriber_.stop();
    active_topic_.clear();
    active_type_.clear();
  }

  /**
   * @brief Get the currently active topic name.
   * @return Active topic name, or empty string if none
   */
  inline const std::string & getActiveTopic() const
  {
    return active_topic_;
  }

  /**
   * @brief Get the currently active message type.
   * @return Active message type, or empty string if none
   */
  inline const std::string & getActiveType() const
  {
    return active_type_;
  }

  /**
   * @brief Check if currently subscribed to a topic.
   * @return true if active subscription exists
   */
  inline bool isSubscribed() const
  {
    return !active_topic_.empty() && !active_type_.empty();
  }

  /**
   * @brief Get count of available topics with supported types.
   * @return Number of supported topics
   */
  inline size_t getTopicCount() const
  {
    return cached_topics_.size();
  }

private:
  AttitudeSubscriber attitude_subscriber_;
  TopicDiscovery topic_discovery_;
  TopicList cached_topics_;
  std::string active_topic_;
  std::string active_type_;
};

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__TOPIC_UTILITIES_HPP_
