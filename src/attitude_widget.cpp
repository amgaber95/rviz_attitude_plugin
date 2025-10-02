/*
 *
 * RViz Attitude Display Plugin - Main Attitude Widget Implementation
 */

#include "rviz_attitude_plugin/attitude_widget.hpp"
#include "rviz_attitude_plugin/euler_converter.hpp"
#include "rviz_attitude_plugin/widgets/attitude_indicator.hpp"
#include "rviz_attitude_plugin/widgets/heading_indicator.hpp"
#include "rviz_attitude_plugin/widgets/angle_readout.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSizePolicy>
#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <cmath>

namespace rviz_attitude_plugin
{

namespace widgets
{

CapsuleFrame::CapsuleFrame(QWidget * parent)
: QWidget(parent)
{
  setAttribute(Qt::WA_TranslucentBackground, true);
}

void CapsuleFrame::paintEvent(QPaintEvent * /*event*/)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  QRectF rf = rect();
  rf.adjust(1.0, 1.0, -1.0, -1.0);

  // Stadium capsule across the full frame
  const double radius = std::min(rf.height() * 0.5, rf.width() * 0.5);
  QPainterPath path;
  path.addRoundedRect(rf, radius, radius);

  // Fill with subtle gradient
  QLinearGradient g(rf.topLeft(), rf.bottomLeft());
  g.setColorAt(0.0, QColor(18, 18, 22, 210));
  g.setColorAt(1.0, QColor(12, 12, 16, 210));
  p.fillPath(path, g);

  // Thin white outline
  p.setPen(QPen(QColor(255, 255, 255, 230), 1.5));
  p.setBrush(Qt::NoBrush);
  p.drawPath(path);
}

}  // namespace widgets

AttitudeWidget::AttitudeWidget(QWidget * parent)
: QWidget(parent),
  display_mode_(DisplayMode::Full),
  show_pitch_ladder_(true),
  show_roll_indicator_(true),
  show_heading_text_(true),
  display_unit_("deg"),
  angles_rad_{{0.0, 0.0, 0.0}},
  angles_deg_{{0.0, 0.0, 0.0}}
{
  buildUI();
}

// Removed previous CapsuleOverlay approach

void AttitudeWidget::buildUI()
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(2, 2, 2, 2);
  layout->setSpacing(1);

  layout->addWidget(buildIndicatorFrame());
  layout->addWidget(buildReadoutFrame());
  layout->addStretch(1);

}
// No custom paint/resizing behavior required

std::string AttitudeWidget::getUnit() const
{
  return display_unit_;
}

void AttitudeWidget::setUnit(const std::string & unit)
{
  if (unit != "deg" && unit != "rad") {
    return;
  }
  display_unit_ = unit;
  refreshReadouts();
}

void AttitudeWidget::setDisplayMode(DisplayMode mode)
{
  display_mode_ = mode;
  updateDisplayMode();
}

void AttitudeWidget::setShowPitchLadder(bool show)
{
  show_pitch_ladder_ = show;
  if (attitude_indicator_) {
    attitude_indicator_->setShowPitchLadder(show);
  }
}

void AttitudeWidget::setShowRollIndicator(bool show)
{
  show_roll_indicator_ = show;
  if (attitude_indicator_) {
    attitude_indicator_->setShowRollIndicator(show);
  }
}

void AttitudeWidget::setShowHeadingText(bool show)
{
  show_heading_text_ = show;
  // Heading text visibility can be implemented in HeadingIndicator if needed
}

void AttitudeWidget::updateDisplayMode()
{
  if (readout_frame_) {
    readout_frame_->setVisible(display_mode_ == DisplayMode::Full);
  }
}

void AttitudeWidget::updateAngles(double roll_rad, double pitch_rad, double yaw_rad)
{
  angles_rad_[0] = roll_rad;
  angles_rad_[1] = pitch_rad;
  angles_rad_[2] = yaw_rad;

  angles_deg_[0] = roll_rad * 180.0 / M_PI;
  angles_deg_[1] = pitch_rad * 180.0 / M_PI;
  angles_deg_[2] = yaw_rad * 180.0 / M_PI;

  attitude_indicator_->setAttitude(angles_deg_[1], angles_deg_[0]);
  heading_->setHeading(angles_deg_[2]);
  refreshReadouts();
}

void AttitudeWidget::refreshReadouts()
{
  const auto & values = (display_unit_ == "deg") ? angles_deg_ : angles_rad_;
  const QString suffix = (display_unit_ == "deg") ? "°" : "";

  roll_readout_->setValue(formatValue(values[0], suffix));
  pitch_readout_->setValue(formatValue(values[1], suffix));
  yaw_readout_->setValue(formatValue(values[2], suffix));
}

QString AttitudeWidget::formatValue(double value, const QString & suffix) const
{
  if (suffix == "°") {
    return QString("%1%2").arg(value, 5, 'f', 1).arg(suffix);
  } else {
    return QString("%1").arg(value, 5, 'f', 3);
  }
}

QWidget * AttitudeWidget::buildIndicatorFrame()
{
  indicator_frame_ = new widgets::CapsuleFrame(this);
  auto * layout = new QHBoxLayout(indicator_frame_);
  layout->setContentsMargins(0, 0, 0 ,0);
  layout->setSpacing(0);

  heading_ = new widgets::HeadingIndicator();
  heading_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  layout->addWidget(heading_);

  attitude_indicator_ = new widgets::AttitudeIndicator();
  attitude_indicator_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  attitude_indicator_->setShowPitchLadder(show_pitch_ladder_);
  attitude_indicator_->setShowRollIndicator(show_roll_indicator_);
  layout->addWidget(attitude_indicator_);

  indicator_container_ = indicator_frame_;
  return indicator_frame_;
}

QWidget * AttitudeWidget::buildReadoutFrame()
{
  readout_frame_ = new QWidget(this);
  auto * layout = new QHBoxLayout(readout_frame_);
  layout->setContentsMargins(2, 2, 2, 2);
  layout->setSpacing(0);

  roll_readout_ = new widgets::AngleReadout("Roll", "#7DD3FC");
  pitch_readout_ = new widgets::AngleReadout("Pitch", "#BBF7D0");
  yaw_readout_ = new widgets::AngleReadout("Yaw", "#FBCFE8");

  for (auto * widget : {roll_readout_, pitch_readout_, yaw_readout_}) {
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addWidget(widget);
  }

  return readout_frame_;
}

}  // namespace rviz_attitude_plugin
