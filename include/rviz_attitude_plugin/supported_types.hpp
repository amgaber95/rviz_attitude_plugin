/*
 * Supported message types and helpers for extracting orientation quaternions.
 */
#ifndef RVIZ_ATTITUDE_PLUGIN__SUPPORTED_TYPES_HPP_
#define RVIZ_ATTITUDE_PLUGIN__SUPPORTED_TYPES_HPP_

#include <string>
#include <string_view>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace rviz_attitude_plugin
{

struct SupportedTypes
{
  static constexpr std::string_view Quaternion = "geometry_msgs/msg/Quaternion";
  static constexpr std::string_view QuaternionStamped = "geometry_msgs/msg/QuaternionStamped";
  static constexpr std::string_view Pose = "geometry_msgs/msg/Pose";
  static constexpr std::string_view PoseStamped = "geometry_msgs/msg/PoseStamped";
  static constexpr std::string_view PoseWithCovariance = "geometry_msgs/msg/PoseWithCovariance";
  static constexpr std::string_view PoseWithCovarianceStamped = "geometry_msgs/msg/PoseWithCovarianceStamped";
  static constexpr std::string_view Imu = "sensor_msgs/msg/Imu";
  static constexpr std::string_view Odometry = "nav_msgs/msg/Odometry";

  static const std::vector<std::string> & list()
  {
    static const std::vector<std::string> kList = {
      std::string(Quaternion),
      std::string(QuaternionStamped),
      std::string(Pose),
      std::string(PoseStamped),
      std::string(PoseWithCovariance),
      std::string(PoseWithCovarianceStamped),
      std::string(Imu),
      std::string(Odometry),
    };
    return kList;
  }

  static bool isSupported(const std::string & type)
  {
    for (const auto & t : list()) {
      if (t == type) return true;
    }
    return false;
  }

  static std::string firstSupported(const std::vector<std::string> & types)
  {
    for (const auto & t : types) {
      if (isSupported(t)) {
        return t;
      }
    }
    return {};
  }
};

inline geometry_msgs::msg::Quaternion extract(const geometry_msgs::msg::Quaternion & msg)
{
  return msg;
}
inline geometry_msgs::msg::Quaternion extract(const geometry_msgs::msg::QuaternionStamped & msg)
{
  return msg.quaternion;
}
inline geometry_msgs::msg::Quaternion extract(const geometry_msgs::msg::Pose & msg)
{
  return msg.orientation;
}
inline geometry_msgs::msg::Quaternion extract(const geometry_msgs::msg::PoseStamped & msg)
{
  return msg.pose.orientation;
}
inline geometry_msgs::msg::Quaternion extract(const geometry_msgs::msg::PoseWithCovariance & msg)
{
  return msg.pose.orientation;
}
inline geometry_msgs::msg::Quaternion extract(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  return msg.pose.pose.orientation;
}
inline geometry_msgs::msg::Quaternion extract(const sensor_msgs::msg::Imu & msg)
{
  return msg.orientation;
}
inline geometry_msgs::msg::Quaternion extract(const nav_msgs::msg::Odometry & msg)
{
  return msg.pose.pose.orientation;
}

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__SUPPORTED_TYPES_HPP_
