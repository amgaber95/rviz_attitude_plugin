/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Main Display Class
 * 
 * Main RViz Display class that coordinates all attitude visualization components.
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__ATTITUDE_DISPLAY_HPP_
#define RVIZ_ATTITUDE_PLUGIN__ATTITUDE_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

#include <memory>

namespace rviz_attitude_plugin
{

// Forward declarations
namespace widgets
{
class AttitudeIndicator;
class HeadingIndicator;
class AngleReadout;
}  // namespace widgets

class OverlayManager;

/**
 * @brief Main RViz Display class for attitude visualization
 * 
 * Integrates all attitude visualization components:
 * - Attitude indicator (pitch/roll with artificial horizon)
 * - Heading indicator (compass rose)
 * - Numeric angle readouts
 * - Overlay rendering system
 */
class AttitudeDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new Attitude Display
   */
  AttitudeDisplay();

  /**
   * @brief Destroy the Attitude Display
   */
  ~AttitudeDisplay() override;

  /**
   * @brief Initialize the display
   */
  void onInitialize() override;

  /**
   * @brief Enable the display
   */
  void onEnable() override;

  /**
   * @brief Disable the display
   */
  void onDisable() override;

  /**
   * @brief Update the display
   * @param wall_dt Time since last update (wall time)
   * @param ros_dt Time since last update (ROS time)
   */
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  /**
   * @brief Update overlay properties when settings change
   */
  void updateOverlayProperties();

private:
  // Overlay rendering system
  std::unique_ptr<OverlayManager> overlay_manager_;

  // Widget components
  std::unique_ptr<widgets::AttitudeIndicator> attitude_widget_;
  std::unique_ptr<widgets::HeadingIndicator> heading_widget_;
  std::unique_ptr<widgets::AngleReadout> pitch_readout_;
  std::unique_ptr<widgets::AngleReadout> roll_readout_;
  std::unique_ptr<widgets::AngleReadout> yaw_readout_;

  // Properties - Display settings
  rviz_common::properties::BoolProperty * show_attitude_property_;
  rviz_common::properties::BoolProperty * show_heading_property_;
  rviz_common::properties::BoolProperty * show_readouts_property_;

  // Properties - Position and size
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * top_property_;

  // Properties - Test values (for now, will be replaced with topic subscription)
  rviz_common::properties::FloatProperty * test_pitch_property_;
  rviz_common::properties::FloatProperty * test_roll_property_;
  rviz_common::properties::FloatProperty * test_yaw_property_;
};

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__ATTITUDE_DISPLAY_HPP_
