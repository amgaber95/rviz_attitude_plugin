/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Artificial Horizon Implementation
 */

#include "rviz_attitude_plugin/widgets/artificial_horizon.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QBrush>
#include <QPen>
#include <QColor>
#include <cmath>
#include <algorithm>

namespace rviz_attitude_plugin
{
namespace widgets
{

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

}  // namespace widgets
}  // namespace rviz_attitude_plugin
