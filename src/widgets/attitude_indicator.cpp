/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Attitude Indicator Components Implementation
 * 
 * Consolidated implementation of all attitude indicator components.
 */

#include "rviz_attitude_plugin/widgets/attitude_indicator.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QPointF>
#include <QPolygonF>
#include <QResizeEvent>
#include <cmath>
#include <algorithm>

namespace rviz_attitude_plugin
{
namespace widgets
{

// ============================================================================
// ArtificialHorizon Implementation
// ============================================================================

ArtificialHorizon::ArtificialHorizon(QWidget * parent)
: QWidget(parent),
  pitch_(0.0),
  roll_(0.0),
  background_visible_(true),
  background_opacity_(1.0)
{
  setMinimumSize(100, 100);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void ArtificialHorizon::setPitchAngle(double pitch)
{
  pitch_ = pitch;
  update();
}

void ArtificialHorizon::setRollAngle(double roll)
{
  roll_ = roll;
  update();
}

void ArtificialHorizon::setAttitude(double pitch, double roll)
{
  pitch_ = pitch;
  roll_ = roll;
  update();
}

void ArtificialHorizon::setBackgroundVisible(bool visible)
{
  background_visible_ = visible;
  update();
}

void ArtificialHorizon::setBackgroundOpacity(double opacity)
{
  background_opacity_ = std::clamp(opacity, 0.0, 1.0);
  update();
}

QSize ArtificialHorizon::sizeHint() const
{
  return QSize(200, 200);
}

void ArtificialHorizon::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const int width = this->width();
  const int height = this->height();
  const int cx = width / 2;
  const int cy = height / 2;
  const double radius = std::min(width, height) / 2.0 - 2;

  if (radius <= 0) {
    return;
  }

  // Clip to circular area
  QPainterPath clip_path;
  clip_path.addEllipse(QPointF(cx, cy), radius, radius);
  painter.setClipPath(clip_path);

  // Translate to center and apply transformations
  painter.translate(cx, cy);
  painter.rotate(roll_);

  // Draw sky/ground
  drawSkyGround(painter, radius);

  // Reset transformations
  painter.resetTransform();
  painter.translate(cx, cy);

  // Draw outer ring
  if (background_visible_) {
    drawOuterRing(painter, radius);
  }
}

void ArtificialHorizon::drawSkyGround(QPainter & painter, double radius)
{
  const double pixels_per_degree = radius / 45.0;
  const double pitch_offset = pitch_ * pixels_per_degree;

  // Sky gradient (blue)
  QLinearGradient sky_gradient(0, -radius, 0, pitch_offset);
  sky_gradient.setColorAt(0.0, QColor(68, 140, 203));   // Light blue
  sky_gradient.setColorAt(1.0, QColor(28, 100, 163));   // Darker blue
  
  // Ground gradient (brown)
  QLinearGradient ground_gradient(0, pitch_offset, 0, radius);
  ground_gradient.setColorAt(0.0, QColor(139, 90, 43));  // Brown
  ground_gradient.setColorAt(1.0, QColor(101, 67, 33));  // Darker brown

  // Draw sky
  QPainterPath sky_path;
  sky_path.addRect(-radius * 2, -radius * 2, radius * 4, radius * 2 + pitch_offset);
  painter.fillPath(sky_path, sky_gradient);

  // Draw ground
  QPainterPath ground_path;
  ground_path.addRect(-radius * 2, pitch_offset, radius * 4, radius * 2 - pitch_offset);
  painter.fillPath(ground_path, ground_gradient);

  // Draw horizon line
  painter.setPen(QPen(Qt::white, 2));
  painter.drawLine(QPointF(-radius * 2, pitch_offset), QPointF(radius * 2, pitch_offset));
}

void ArtificialHorizon::drawOuterRing(QPainter & painter, double radius)
{
  // Draw subtle outer ring border
  painter.setPen(QPen(QColor(255, 255, 255, static_cast<int>(background_opacity_ * 100)), 2));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(QPointF(0, 0), radius, radius);
}

// ============================================================================
// AircraftReference Implementation
// ============================================================================

AircraftReference::AircraftReference(QWidget * parent)
: QWidget(parent),
  color_(Qt::yellow)
{
  setMinimumSize(50, 50);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setAttribute(Qt::WA_TranslucentBackground);
}

void AircraftReference::setColor(const QColor & color)
{
  color_ = color;
  update();
}

void AircraftReference::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const int width = this->width();
  const int height = this->height();
  const int cx = width / 2;
  const int cy = height / 2;

  const double scale = std::min(width, height) / 200.0;
  const double wing_length = 40 * scale;
  const double wing_thickness = 3 * scale;
  const double tick_height = 8 * scale;
  const double center_dot_radius = 3 * scale;

  QPen pen(color_, wing_thickness);
  pen.setCapStyle(Qt::RoundCap);
  painter.setPen(pen);
  painter.setBrush(color_);

  // Draw center dot
  painter.drawEllipse(QPointF(cx, cy), center_dot_radius, center_dot_radius);

  // Draw left wing
  painter.drawLine(QPointF(cx - wing_length, cy), QPointF(cx - center_dot_radius, cy));
  
  // Draw right wing
  painter.drawLine(QPointF(cx + center_dot_radius, cy), QPointF(cx + wing_length, cy));

  // Draw left wing tick (vertical)
  painter.drawLine(QPointF(cx - wing_length, cy - tick_height), 
                   QPointF(cx - wing_length, cy));

  // Draw right wing tick (vertical)
  painter.drawLine(QPointF(cx + wing_length, cy - tick_height), 
                   QPointF(cx + wing_length, cy));
}

// ============================================================================
// RollIndicator Implementation
// ============================================================================

RollIndicator::RollIndicator(QWidget * parent)
: QWidget(parent),
  roll_(0.0)
{
  setMinimumSize(100, 100);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setAttribute(Qt::WA_TranslucentBackground);
}

void RollIndicator::setRollAngle(double roll)
{
  roll_ = roll;
  update();
}

void RollIndicator::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const int width = this->width();
  const int height = this->height();
  const int cx = width / 2;
  const int cy = height / 2;
  const double radius = std::min(width, height) / 2.0 - 5;

  if (radius <= 10) {
    return;
  }

  painter.translate(cx, cy);

  // Draw arc
  const int start_angle = 30 * 16;  // 30° from top (Qt uses 1/16th degree units)
  const int span_angle = 120 * 16;  // 120° span (60° on each side)
  
  QPen arc_pen(QColor(255, 255, 255, 180), 2);
  painter.setPen(arc_pen);
  painter.setBrush(Qt::NoBrush);
  painter.drawArc(QRectF(-radius, -radius, radius * 2, radius * 2), start_angle, span_angle);

  // Draw tick marks
  const double tick_inner = radius - 8;
  const double tick_outer = radius;
  const double major_tick_inner = radius - 12;

  // Major ticks at 0°, 30°, 45°, 60° (both sides)
  QPen major_pen(QColor(255, 255, 255, 230), 2.5);
  painter.setPen(major_pen);
  
  for (int angle : {0, 30, 45, 60}) {
    // Right side
    double rad = (90 - angle) * M_PI / 180.0;
    painter.drawLine(
      QPointF(std::cos(rad) * major_tick_inner, -std::sin(rad) * major_tick_inner),
      QPointF(std::cos(rad) * tick_outer, -std::sin(rad) * tick_outer)
    );
    
    // Left side (if not center)
    if (angle != 0) {
      rad = (90 + angle) * M_PI / 180.0;
      painter.drawLine(
        QPointF(std::cos(rad) * major_tick_inner, -std::sin(rad) * major_tick_inner),
        QPointF(std::cos(rad) * tick_outer, -std::sin(rad) * tick_outer)
      );
    }
  }

  // Minor ticks at 10°, 20° intervals
  QPen minor_pen(QColor(255, 255, 255, 150), 1.5);
  painter.setPen(minor_pen);
  
  for (int angle : {10, 20, 50}) {
    // Right side
    double rad = (90 - angle) * M_PI / 180.0;
    painter.drawLine(
      QPointF(std::cos(rad) * tick_inner, -std::sin(rad) * tick_inner),
      QPointF(std::cos(rad) * tick_outer, -std::sin(rad) * tick_outer)
    );
    
    // Left side
    rad = (90 + angle) * M_PI / 180.0;
    painter.drawLine(
      QPointF(std::cos(rad) * tick_inner, -std::sin(rad) * tick_inner),
      QPointF(std::cos(rad) * tick_outer, -std::sin(rad) * tick_outer)
    );
  }

  // Draw rotating pointer triangle
  painter.rotate(roll_);
  
  const double triangle_base = 8;
  const double triangle_height = 12;
  const double pointer_y = -radius + 2;
  
  QPolygonF triangle;
  triangle << QPointF(0, pointer_y)
           << QPointF(-triangle_base / 2, pointer_y + triangle_height)
           << QPointF(triangle_base / 2, pointer_y + triangle_height);
  
  painter.setPen(QPen(Qt::yellow, 1.5));
  painter.setBrush(Qt::yellow);
  painter.drawPolygon(triangle);
}

// ============================================================================
// AttitudeIndicator Implementation
// ============================================================================

AttitudeIndicator::AttitudeIndicator(QWidget * parent)
: QWidget(parent),
  pitch_(0.0),
  roll_(0.0),
  show_roll_indicator_(true)
{
  setMinimumSize(150, 150);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // Create child widgets (stacked in order)
  horizon_ = new ArtificialHorizon(this);
  aircraft_ = new AircraftReference(this);
  roll_indicator_ = new RollIndicator(this);

  // Set initial geometry
  updateChildGeometry();
}

void AttitudeIndicator::setPitchAngle(double pitch)
{
  pitch_ = pitch;
  if (horizon_) {
    horizon_->setPitchAngle(pitch);
  }
}

void AttitudeIndicator::setRollAngle(double roll)
{
  roll_ = roll;
  if (horizon_) {
    horizon_->setRollAngle(roll);
  }
  if (roll_indicator_) {
    roll_indicator_->setRollAngle(roll);
  }
}

void AttitudeIndicator::setAttitude(double pitch, double roll)
{
  pitch_ = pitch;
  roll_ = roll;
  
  if (horizon_) {
    horizon_->setAttitude(pitch, roll);
  }
  if (roll_indicator_) {
    roll_indicator_->setRollAngle(roll);
  }
}

void AttitudeIndicator::setShowRollIndicator(bool show)
{
  show_roll_indicator_ = show;
  if (roll_indicator_) {
    roll_indicator_->setVisible(show);
  }
}

QSize AttitudeIndicator::sizeHint() const
{
  return QSize(200, 200);
}

void AttitudeIndicator::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  updateChildGeometry();
}

void AttitudeIndicator::updateChildGeometry()
{
  const QRect rect = this->rect();
  
  // All child widgets occupy the full area (stacked on top of each other)
  if (horizon_) {
    horizon_->setGeometry(rect);
  }
  if (aircraft_) {
    aircraft_->setGeometry(rect);
  }
  if (roll_indicator_) {
    roll_indicator_->setGeometry(rect);
  }

  // Ensure proper stacking order (back to front)
  if (horizon_) {
    horizon_->lower();
  }
  if (roll_indicator_) {
    roll_indicator_->raise();
  }
  if (aircraft_) {
    aircraft_->raise();  // Aircraft on top
  }
}

}  // namespace widgets
}  // namespace rviz_attitude_plugin
