/*
 *
 * RViz Attitude Display - Main Display Plugin Class
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__ATTITUDE_DISPLAY_HPP_
#define RVIZ_ATTITUDE_PLUGIN__ATTITUDE_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <QEvent>

#include <rclcpp/rclcpp.hpp>
#include "rviz_attitude_plugin/topic_utilities.hpp"
#include "rviz_attitude_plugin/overlay_system.hpp"

#include <memory>
#include <string>
#include <vector>
#include <array>

namespace rviz_common
{
class RenderPanel;
}

class QWidget;

namespace rviz_attitude_plugin
{

class AttitudeWidget;
class EulerConverter;
class OverlayPanel;
class OverlayManager;

/**
 * @brief RViz display plugin for visualizing attitude/orientation data.
 * 
 * Displays attitude information from various ROS messages containing quaternions:
 * - geometry_msgs/QuaternionStamped
 * - geometry_msgs/PoseStamped  
 * - geometry_msgs/PoseWithCovarianceStamped
 * - sensor_msgs/Imu
 * - nav_msgs/Odometry
 * 
 * Shows:
 * - Artificial horizon (pitch/roll)
 * - Heading indicator (yaw)
 * - Numeric Euler angle readouts
 * - Multiple Euler convention support
 */
class AttitudeDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  AttitudeDisplay();
  ~AttitudeDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void updateAngleUnit();
  void updateDisplayMode();
  void updateOverlayProperties();
  void onRefreshTopics();
  void onTopicChanged();

private:
  void setupProperties();
  void updateDisplay(double x, double y, double z, double w);
  void attachOverlay();
  bool eventFilter(QObject * object, QEvent * event) override;
  void refreshSupportedTopics();
  void subscribeToSelected();
  // Unified handler for normalized orientation messages
  void onOrientation(const geometry_msgs::msg::Quaternion & q);

  std::unique_ptr<EulerConverter> converter_;
  std::unique_ptr<AttitudeWidget> widget_;

  // Properties
  rviz_common::properties::EnumProperty * topic_property_;
  rviz_common::properties::BoolProperty * refresh_button_property_;
  rviz_common::properties::StringProperty * current_type_property_;
  rviz_common::properties::IntProperty * overlay_width_property_;
  rviz_common::properties::IntProperty * overlay_height_property_;
  rviz_common::properties::BoolProperty * show_overlay_property_;
  rviz_common::properties::IntProperty * overlay_x_property_;
  rviz_common::properties::IntProperty * overlay_y_property_;
  rviz_common::properties::EnumProperty * overlay_anchor_property_;
  rviz_common::properties::EnumProperty * angle_unit_property_;
  rviz_common::properties::EnumProperty * display_mode_property_;

  // State
  std::array<double, 4> last_quaternion_;  // x, y, z, w
  bool has_data_;
  rviz_common::RenderPanel * render_panel_;
  std::unique_ptr<OverlayManager> overlay_manager_;
  bool overlay_event_filter_installed_;

  // Managers for separated concerns
  AttitudeTopicManager topic_manager_;
  OverlayGeometryManager geometry_manager_;
  std::vector<std::string> topic_options_;
};

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__ATTITUDE_DISPLAY_HPP_
