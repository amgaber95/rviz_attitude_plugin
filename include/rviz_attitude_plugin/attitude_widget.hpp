/*
 * RViz Attitude Display Plugin - Main Attitude Widget
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__ATTITUDE_WIDGET_HPP_
#define RVIZ_ATTITUDE_PLUGIN__ATTITUDE_WIDGET_HPP_

#include <QWidget>
#include <QString>
#include <memory>
#include <array>

namespace rviz_attitude_plugin
{
namespace widgets
{
class AttitudeIndicator;
class HeadingIndicator;
class AngleReadout;

/**
 * @brief Frame widget with capsule/rounded background styling
 */
class CapsuleFrame : public QWidget
{
  Q_OBJECT
public:
  explicit CapsuleFrame(QWidget * parent = nullptr);
protected:
  void paintEvent(QPaintEvent * event) override;
};

}  // namespace widgets

/**
 * @brief Display mode for the attitude widget.
 */
enum class DisplayMode
{
  Full,     // Heading, attitude, and angle readouts (complete information)
  Compact   // Heading and attitude only (minimal/compact display)
};

/**
 * @brief Main widget combining all attitude visualization components.
 * 
 * This widget provides:
 * - Topic selection and refresh
 * - Euler convention selection
 * - Unit toggle (degrees/radians)
 * - Attitude indicator display (pitch, roll)
 * - Heading indicator
 * - Numeric readouts for roll, pitch, yaw (in Full mode)
 * 
 * Supports two display modes:
 * - Full: Shows heading, attitude, and numeric angle readouts (complete information)
 * - Compact: Shows heading and attitude indicators only (minimal display)
 */
class AttitudeWidget : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(DisplayMode displayMode READ displayMode WRITE setDisplayMode)
  Q_PROPERTY(bool showPitchLadder READ showPitchLadder WRITE setShowPitchLadder)
  Q_PROPERTY(bool showRollIndicator READ showRollIndicator WRITE setShowRollIndicator)
  Q_PROPERTY(bool showHeadingText READ showHeadingText WRITE setShowHeadingText)

public:
  explicit AttitudeWidget(QWidget * parent = nullptr);
  ~AttitudeWidget() override = default;

  // Display mode
  DisplayMode displayMode() const { return display_mode_; }
  void setDisplayMode(DisplayMode mode);

  // Property getters
  bool showPitchLadder() const { return show_pitch_ladder_; }
  bool showRollIndicator() const { return show_roll_indicator_; }
  bool showHeadingText() const { return show_heading_text_; }

  // Property setters
  void setShowPitchLadder(bool show);
  void setShowRollIndicator(bool show);
  void setShowHeadingText(bool show);

  // Configuration
  std::string getUnit() const;
  void setUnit(const std::string & unit);

  // Update visualization
  void updateAngles(double roll_rad, double pitch_rad, double yaw_rad);

private:
  void buildUI();
  QWidget * buildIndicatorFrame();
  QWidget * buildReadoutFrame();
  void refreshReadouts();
  void updateDisplayMode();
  QString formatValue(double value, const QString & suffix) const;

  // UI components
  QWidget * indicator_container_;
  widgets::AttitudeIndicator * attitude_indicator_;
  widgets::HeadingIndicator * heading_;
  widgets::AngleReadout * roll_readout_;
  widgets::AngleReadout * pitch_readout_;
  widgets::AngleReadout * yaw_readout_;
  widgets::CapsuleFrame * indicator_frame_;
  QWidget * readout_frame_;

  // State
  DisplayMode display_mode_;
  bool show_pitch_ladder_;
  bool show_roll_indicator_;
  bool show_heading_text_;
  std::string display_unit_;
  std::array<double, 3> angles_rad_;  // roll, pitch, yaw
  std::array<double, 3> angles_deg_;  // roll, pitch, yaw
};

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__ATTITUDE_WIDGET_HPP_
