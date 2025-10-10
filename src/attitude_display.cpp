/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display - Main Display Plugin Implementation
 */

#include "rviz_attitude_plugin/attitude_display.hpp"
#include "rviz_attitude_plugin/attitude_widget.hpp"
#include "rviz_attitude_plugin/euler_converter.hpp"
#include "rviz_attitude_plugin/overlay_system.hpp"
#include "rviz_attitude_plugin/supported_types.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_rendering/render_system.hpp>

#include <algorithm>
#include <QColor>
#include <QEvent>
#include <QPainter>
#include <QPoint>
#include <QSize>
#include <QTimer>

namespace rviz_attitude_plugin
{

// Timing constants for async operations
static constexpr int TOPIC_DISCOVERY_DELAY_MS = 250;
static constexpr int BUTTON_RESET_DELAY_MS = 100;

AttitudeDisplay::AttitudeDisplay()
: last_quaternion_{{0.0, 0.0, 0.0, 1.0}},
  has_data_(false),
  render_panel_(nullptr),
  overlay_event_filter_installed_(false)
{
  setupProperties();
}

AttitudeDisplay::~AttitudeDisplay()
{
  if (render_panel_ && overlay_event_filter_installed_) {
    render_panel_->removeEventFilter(this);
  }

  if (widget_) {
    widget_->hide();
    widget_.reset();
  }
}

void AttitudeDisplay::setupProperties()
{
  // Overlay properties
  show_overlay_property_ = new rviz_common::properties::BoolProperty(
    "Show Overlay",
    true,
    "Show the attitude widget as an overlay",
    this,
    SLOT(updateOverlayProperties()));

  // Topic selection and refresh
  topic_property_ = new rviz_common::properties::EnumProperty(
    "Topic",
    "",
    "Topics publishing supported message types",
    this,
    SLOT(onTopicChanged()));

  refresh_button_property_ = new rviz_common::properties::BoolProperty(
    "Refresh Topics",
    false,
    "Click to refresh the list of supported topics",
    this,
    SLOT(onRefreshTopics()));

  current_type_property_ = new rviz_common::properties::StringProperty(
    "Current Type",
    "",
    "Type of the currently selected topic",
    this);

  overlay_x_property_ = new rviz_common::properties::IntProperty(
    "Overlay X",
    16,
    "Horizontal offset in pixels from the render view origin",
    show_overlay_property_,
    SLOT(updateOverlayProperties()),
    this);
  overlay_x_property_->setMin(0);

  overlay_y_property_ = new rviz_common::properties::IntProperty(
    "Overlay Y",
    16,
    "Vertical offset in pixels from the render view origin",
    show_overlay_property_,
    SLOT(updateOverlayProperties()),
    this);
  overlay_y_property_->setMin(0);

  overlay_anchor_property_ = new rviz_common::properties::EnumProperty(
    "Overlay Anchor",
    "Bottom Right",
    "Which screen corner acts as the anchor for the overlay offsets",
    show_overlay_property_,
    SLOT(updateOverlayProperties()),
    this);
  overlay_anchor_property_->addOption("Top Right", 0);
  overlay_anchor_property_->addOption("Top Left", 1);
  overlay_anchor_property_->addOption("Bottom Right", 2);
  overlay_anchor_property_->addOption("Bottom Left", 3);
  overlay_anchor_property_->setValue("Bottom Right");

  overlay_width_property_ = new rviz_common::properties::IntProperty(
    "Overlay Width",
    320,
    "Width of the overlay widget",
    show_overlay_property_,
    SLOT(updateOverlayProperties()),
    this);
  overlay_width_property_->setMin(200);
  overlay_width_property_->setMax(800);

  overlay_height_property_ = new rviz_common::properties::IntProperty(
    "Overlay Height",
    240,
    "Height of the overlay widget",
    show_overlay_property_,
    SLOT(updateOverlayProperties()),
    this);
  overlay_height_property_->setMin(160);
  overlay_height_property_->setMax(800);

  angle_unit_property_ = new rviz_common::properties::EnumProperty(
    "Angle Units",
    "",
    "Units used for roll/pitch/yaw readouts",
    this,
    SLOT(updateAngleUnit()));
  angle_unit_property_->addOption("Degrees", 0);
  angle_unit_property_->addOption("Radians", 1);
  angle_unit_property_->setString("Degrees"); // default to Degrees

  display_mode_property_ = new rviz_common::properties::EnumProperty(
    "Display Mode",
    "",
    "Display mode for the attitude widget (Full: heading+attitude+readouts, Compact: heading+attitude only)",
    this,
    SLOT(updateDisplayMode()));
  display_mode_property_->addOption("Full", 0);
  display_mode_property_->addOption("Compact", 1);
  display_mode_property_->setString("Full"); // default to Full

  // No background toggles in properties; defaults are set in onInitialize
}

void AttitudeDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();

  // Ensure a custom icon shows up in the Displays tree; this matches
  // RViz's default lookup of icons/classes/<DisplayName>.{svg,png}.
  setIcon(rviz_common::loadPixmap("package://rviz_attitude_plugin/icons/classes/Attitude.png"));

  converter_ = std::make_unique<EulerConverter>();
  widget_ = std::make_unique<AttitudeWidget>();

  widget_->setVisible(false);
  widget_->setAttribute(Qt::WA_ShowWithoutActivating);
  widget_->setAttribute(Qt::WA_TranslucentBackground, true);
  widget_->setFocusPolicy(Qt::NoFocus);
  widget_->setAutoFillBackground(false);
  // Use a custom painted background in AttitudeWidget; no stylesheet background.
  widget_->setStyleSheet("");

  const int unit_index = angle_unit_property_->getOptionInt();
  widget_->setUnit(unit_index == 0 ? std::string("deg") : std::string("rad"));

  attachOverlay();
  updateOverlayProperties();

  // Populate topics initially
  refreshSupportedTopics();
  // And again shortly after startup to catch late discovery
  QTimer::singleShot(TOPIC_DISCOVERY_DELAY_MS, [this]() { refreshSupportedTopics(); });
}

void AttitudeDisplay::onEnable()
{
  rviz_common::Display::onEnable();
  attachOverlay();
  if (show_overlay_property_->getBool()) {
    if (overlay_manager_) {
      overlay_manager_->setVisible(true);
      overlay_manager_->render(*widget_);
    }
  }
}

void AttitudeDisplay::onDisable()
{
  rviz_common::Display::onDisable();
  if (overlay_manager_) overlay_manager_->setVisible(false);
  topic_manager_.unsubscribe();
}


void AttitudeDisplay::updateDisplay(double x, double y, double z, double w)
{
  last_quaternion_[0] = x;
  last_quaternion_[1] = y;
  last_quaternion_[2] = z;
  last_quaternion_[3] = w;
  has_data_ = true;

  double roll, pitch, yaw;
  converter_->convert(x, y, z, w, roll, pitch, yaw);

  if (widget_) {
    widget_->updateAngles(roll, pitch, yaw);
    if (overlay_manager_) overlay_manager_->render(*widget_);
  }
}

void AttitudeDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  // Periodically raise widget to keep it on top
  if (widget_ && widget_->isVisible() && show_overlay_property_->getBool()) {
    widget_->raise();
  }
}

// No updateEulerConvention: always use ROS tf2 RPY conversion

// No background-toggle slots; handled by defaults

void AttitudeDisplay::updateAngleUnit()
{
  const int unit_index = angle_unit_property_->getOptionInt();
  const std::string unit = (unit_index == 0) ? "deg" : "rad";
  if (widget_) {
    widget_->setUnit(unit);
    if (overlay_manager_) overlay_manager_->render(*widget_);
  }

  if (has_data_) {
    updateDisplay(last_quaternion_[0], last_quaternion_[1], last_quaternion_[2], last_quaternion_[3]);
  }
}

void AttitudeDisplay::updateDisplayMode()
{
  const int mode_index = display_mode_property_->getOptionInt();
  const rviz_attitude_plugin::DisplayMode mode = (mode_index == 0) 
    ? rviz_attitude_plugin::DisplayMode::Full 
    : rviz_attitude_plugin::DisplayMode::Compact;
  if (widget_) {
    widget_->setDisplayMode(mode);
    if (overlay_manager_) overlay_manager_->render(*widget_);
  }
}

void AttitudeDisplay::updateOverlayProperties()
{
  attachOverlay();

  const int width = overlay_width_property_->getInt();
  const int height = overlay_height_property_->getInt();
  const int offset_x = overlay_x_property_->getInt();
  const int offset_y = overlay_y_property_->getInt();
  const int anchor_index = overlay_anchor_property_->getOptionInt();
  const auto anchor = static_cast<OverlayGeometryManager::Anchor>(anchor_index);
  const bool show = show_overlay_property_->getBool() && isEnabled();

  // Use OverlayGeometryManager to handle positioning
  geometry_manager_.setGeometry(width, height, offset_x, offset_y, anchor);

  if (overlay_manager_ && render_panel_) {
    const QSize panel_size = render_panel_->size();
    auto [clamped_x, clamped_y] = geometry_manager_.calculateClampedOffsets(panel_size);

    overlay_manager_->setGeometry(width, height, clamped_x, clamped_y, anchor);
    if (widget_) overlay_manager_->render(*widget_);
    overlay_manager_->setVisible(show);
  }
}

void AttitudeDisplay::attachOverlay()
{
  if (!context_) {
    return;
  }

  if (!overlay_manager_) overlay_manager_ = std::make_unique<OverlayManager>();
  overlay_manager_->attach(context_);
  render_panel_ = overlay_manager_->getRenderPanel();

  if (render_panel_ && !overlay_event_filter_installed_) {
    render_panel_->installEventFilter(this);
    overlay_event_filter_installed_ = true;
  }
}

bool AttitudeDisplay::eventFilter(QObject * object, QEvent * event)
{
  if (object == render_panel_) {
    if (event->type() == QEvent::Resize || event->type() == QEvent::Show) {
      updateOverlayProperties();
    }
  }

  return QObject::eventFilter(object, event);
}

void AttitudeDisplay::onRefreshTopics()
{
  refreshSupportedTopics();
  if (refresh_button_property_) {
    QTimer::singleShot(BUTTON_RESET_DELAY_MS, [this]() {
      if (refresh_button_property_) {
        refresh_button_property_->setBool(false);
      }
    });
  }
}

void AttitudeDisplay::onTopicChanged()
{
  topic_manager_.unsubscribe();
  subscribeToSelected();
}

void AttitudeDisplay::refreshSupportedTopics()
{
  if (!context_) return;
  auto ros_node = context_->getRosNodeAbstraction().lock();
  if (!ros_node) return;
  auto node = ros_node->get_raw_node();

  // Use TopicManager to refresh topics
  auto items = topic_manager_.refreshTopics(node.get());
  size_t count = items.size();

  topic_property_->clearOptions();
  topic_options_.clear();

  for (const auto & [topic, type] : items) {
    topic_property_->addOption(QString::fromStdString(topic));
    topic_options_.push_back(topic);
  }

  // Keep current selection if still present
  const std::string active_topic = topic_manager_.getActiveTopic();
  if (!active_topic.empty()) {
    topic_property_->setString(QString::fromStdString(active_topic));
  }

  // Auto-select if nothing selected
  if (active_topic.empty() && !topic_options_.empty()) {
    const std::string new_topic = topic_options_.front();
    topic_property_->setString(QString::fromStdString(new_topic));
    subscribeToSelected();
  }

  // Status feedback
  if (count == 0) {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Topics", "No supported topics found");
  } else {
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topics",
      QString("%1 supported topic(s)").arg(static_cast<int>(count)));
  }
}

void AttitudeDisplay::onOrientation(const geometry_msgs::msg::Quaternion & q)
{
  updateDisplay(q.x, q.y, q.z, q.w);
}

void AttitudeDisplay::subscribeToSelected()
{
  if (!context_) return;
  auto ros_node = context_->getRosNodeAbstraction().lock();
  if (!ros_node) return;
  auto node = ros_node->get_raw_node();

  const std::string topic = topic_property_->getStdString();
  if (topic.empty()) return;

  // Use TopicManager to resolve type and subscribe
  const std::string type = topic_manager_.resolveType(node.get(), topic);
  current_type_property_->setString(QString::fromStdString(type));
  if (type.empty()) return;

  // Subscribe using TopicManager
  topic_manager_.subscribe(node.get(), topic, type,
    [this](const geometry_msgs::msg::Quaternion & q){ onOrientation(q); });
}

// Support for other message types via template specialization would go here
// For now, the primary message type is QuaternionStamped

}  // namespace rviz_attitude_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_attitude_plugin::AttitudeDisplay, rviz_common::Display)
