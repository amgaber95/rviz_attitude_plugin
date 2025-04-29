/*
 * RViz Attitude Display Plugin - Topic Discovery Utilities (Header-Only)
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__TOPIC_UTILITIES_HPP_
#define RVIZ_ATTITUDE_PLUGIN__TOPIC_UTILITIES_HPP_

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

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

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__TOPIC_UTILITIES_HPP_

