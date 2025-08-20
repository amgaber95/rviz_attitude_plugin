/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Main Display Implementation
 */

#include "rviz_attitude_plugin/attitude_display.hpp"
#include "rviz_attitude_plugin/overlay_system.hpp"
#include "rviz_attitude_plugin/widgets/attitude_indicator.hpp"
#include "rviz_attitude_plugin/widgets/heading_indicator.hpp"
#include "rviz_attitude_plugin/widgets/angle_readout.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace rviz_attitude_plugin
{

AttitudeDisplay::AttitudeDisplay()
: Display()
{
  // Display component properties
  show_attitude_property_ = new rviz_common::properties::BoolProperty(
    "Show Attitude Indicator", true,
    "Show the attitude indicator (artificial horizon)",
    this, SLOT(updateOverlayProperties()));

  show_heading_property_ = new rviz_common::properties::BoolProperty(
    "Show Heading Indicator", true,
    "Show the heading indicator (compass rose)",
    this, SLOT(updateOverlayProperties()));

  show_readouts_property_ = new rviz_common::properties::BoolProperty(
    "Show Numeric Readouts", true,
    "Show numeric angle readouts",
    this, SLOT(updateOverlayProperties()));

  // Position and size properties
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 400,
    "Width of the overlay display in pixels",
    this, SLOT(updateOverlayProperties()));
  width_property_->setMin(100);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 300,
    "Height of the overlay display in pixels",
    this, SLOT(updateOverlayProperties()));
  height_property_->setMin(100);

  left_property_ = new rviz_common::properties::IntProperty(
    "Left", 10,
    "Left position of the overlay in pixels",
    this, SLOT(updateOverlayProperties()));
  left_property_->setMin(0);

  top_property_ = new rviz_common::properties::IntProperty(
    "Top", 10,
    "Top position of the overlay in pixels",
    this, SLOT(updateOverlayProperties()));
  top_property_->setMin(0);

  // Test value properties (temporary, for testing without topics)
  test_pitch_property_ = new rviz_common::properties::FloatProperty(
    "Test Pitch", 0.0,
    "Test pitch angle in degrees (for testing)",
    this, SLOT(updateOverlayProperties()));
  test_pitch_property_->setMin(-90.0);
  test_pitch_property_->setMax(90.0);

  test_roll_property_ = new rviz_common::properties::FloatProperty(
    "Test Roll", 0.0,
    "Test roll angle in degrees (for testing)",
    this, SLOT(updateOverlayProperties()));
  test_roll_property_->setMin(-180.0);
  test_roll_property_->setMax(180.0);

  test_yaw_property_ = new rviz_common::properties::FloatProperty(
    "Test Yaw", 0.0,
    "Test yaw/heading angle in degrees (for testing)",
    this, SLOT(updateOverlayProperties()));
  test_yaw_property_->setMin(0.0);
  test_yaw_property_->setMax(360.0);
}

AttitudeDisplay::~AttitudeDisplay()
{
  onDisable();
}

void AttitudeDisplay::onInitialize()
{
  // Create widget components first
  attitude_widget_ = std::make_unique<widgets::AttitudeIndicator>();
  heading_widget_ = std::make_unique<widgets::HeadingIndicator>();
  
  pitch_readout_ = std::make_unique<widgets::AngleReadout>();
  pitch_readout_->setTitle("Pitch");
  pitch_readout_->setColor(QColor(0, 255, 0));
  
  roll_readout_ = std::make_unique<widgets::AngleReadout>();
  roll_readout_->setTitle("Roll");
  roll_readout_->setColor(QColor(0, 200, 255));
  
  yaw_readout_ = std::make_unique<widgets::AngleReadout>();
  yaw_readout_->setTitle("Heading");
  yaw_readout_->setColor(QColor(255, 200, 0));

  // Create overlay manager
  overlay_manager_ = std::make_unique<OverlayManager>();
  
  // Initialize overlay with context, widget, and name
  overlay_manager_->initialize(context_, attitude_widget_.get(), "AttitudeOverlay");

  setStatus(rviz_common::properties::StatusProperty::Ok, "Overlay", "Initialized");
}

void AttitudeDisplay::onEnable()
{
  if (overlay_manager_) {
    updateOverlayProperties();
  }
}

void AttitudeDisplay::onDisable()
{
  if (overlay_manager_) {
    overlay_manager_->shutdown();
  }
}

void AttitudeDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  if (!overlay_manager_ || !isEnabled()) {
    return;
  }

  // Update widget values with test properties
  if (attitude_widget_) {
    attitude_widget_->setPitchAngle(test_pitch_property_->getFloat());
    attitude_widget_->setRollAngle(test_roll_property_->getFloat());
  }

  if (heading_widget_) {
    heading_widget_->setHeadingAngle(test_yaw_property_->getFloat());
  }

  if (pitch_readout_) {
    pitch_readout_->setValue(test_pitch_property_->getFloat());
  }

  if (roll_readout_) {
    roll_readout_->setValue(test_roll_property_->getFloat());
  }

  if (yaw_readout_) {
    yaw_readout_->setValue(test_yaw_property_->getFloat());
  }

  // Render the overlay
  if (show_attitude_property_->getBool()) {
    overlay_manager_->render();
  }
}

void AttitudeDisplay::updateOverlayProperties()
{
  if (!overlay_manager_) {
    return;
  }

  // Update overlay position and size
  // Using TopLeft anchor and visible=true as defaults
  overlay_manager_->updateProperties(
    width_property_->getInt(),
    height_property_->getInt(),
    left_property_->getInt(),
    top_property_->getInt(),
    OverlayGeometryManager::Anchor::TopLeft,
    show_attitude_property_->getBool()
  );
}

}  // namespace rviz_attitude_plugin

// Register this display class as a plugin
PLUGINLIB_EXPORT_CLASS(rviz_attitude_plugin::AttitudeDisplay, rviz_common::Display)
