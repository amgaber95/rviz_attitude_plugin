/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Heading Indicator Implementation
 */

#include "rviz_attitude_plugin/widgets/heading_indicator.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QPointF>
#include <QPolygonF>
#include <QFont>
#include <QFontMetrics>
#include <cmath>
#include <algorithm>

namespace rviz_attitude_plugin
{
namespace widgets
{

HeadingIndicator::HeadingIndicator(QWidget * parent)
: QWidget(parent),
  heading_(0.0),
  show_text_(true)
{
  setMinimumSize(150, 150);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setAttribute(Qt::WA_TranslucentBackground);
}

void HeadingIndicator::setHeadingAngle(double heading)
{
  heading_ = normalizeAngle(heading);
  update();
}

void HeadingIndicator::setShowText(bool show)
{
  show_text_ = show;
  update();
}

QSize HeadingIndicator::sizeHint() const
{
  return QSize(200, 200);
}

void HeadingIndicator::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const int width = this->width();
  const int height = this->height();
  const int cx = width / 2;
  const int cy = height / 2;
  const double radius = std::min(width, height) / 2.0 - 10;

  if (radius <= 20) {
    return;
  }

  painter.translate(cx, cy);

  // Draw compass rose (rotated)
  drawCompassRose(painter, radius);

  // Draw direction labels (rotated with compass)
  drawDirectionLabels(painter, radius);

  // Draw degree markers (rotated with compass)
  drawDegreeMarkers(painter, radius);

  // Reset rotation for fixed elements
  painter.resetTransform();
  painter.translate(cx, cy);

  // Draw fixed heading pointer at top
  drawHeadingPointer(painter, radius);

  // Draw heading text if enabled
  if (show_text_) {
    painter.resetTransform();
    drawHeadingText(painter);
  }
}

void HeadingIndicator::drawCompassRose(QPainter & painter, double radius)
{
  // Rotate compass rose based on heading (counter-clockwise)
  painter.rotate(-heading_);

  // Draw outer circle
  QPen outer_pen(QColor(255, 255, 255, 200), 2);
  painter.setPen(outer_pen);
  painter.setBrush(QColor(0, 0, 0, 100));
  painter.drawEllipse(QPointF(0, 0), radius, radius);

  // Draw inner circle
  const double inner_radius = radius * 0.85;
  QPen inner_pen(QColor(255, 255, 255, 150), 1.5);
  painter.setPen(inner_pen);
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(QPointF(0, 0), inner_radius, inner_radius);
}

void HeadingIndicator::drawDirectionLabels(QPainter & painter, double radius)
{
  const double label_radius = radius * 0.70;
  
  // Cardinal and intermediate directions
  struct Direction {
    double angle;      // Degrees from North (clockwise)
    QString label;
    bool is_cardinal;  // Cardinal directions use larger, bolder text
  };

  const Direction directions[] = {
    {0.0, "N", true},
    {45.0, "NE", false},
    {90.0, "E", true},
    {135.0, "SE", false},
    {180.0, "S", true},
    {225.0, "SW", false},
    {270.0, "W", true},
    {315.0, "NW", false}
  };

  for (const auto & dir : directions) {
    const double angle_rad = (dir.angle - 90.0) * M_PI / 180.0;
    const double x = std::cos(angle_rad) * label_radius;
    const double y = std::sin(angle_rad) * label_radius;

    // Set font based on cardinal vs intermediate
    QFont font;
    if (dir.is_cardinal) {
      font.setPixelSize(static_cast<int>(radius * 0.15));
      font.setBold(true);
      painter.setPen(QPen(Qt::white, 2));
    } else {
      font.setPixelSize(static_cast<int>(radius * 0.11));
      font.setBold(false);
      painter.setPen(QPen(QColor(200, 200, 200), 1.5));
    }
    painter.setFont(font);

    // Center text at position
    QFontMetrics metrics(font);
    const int text_width = metrics.horizontalAdvance(dir.label);
    const int text_height = metrics.height();
    painter.drawText(
      static_cast<int>(x - text_width / 2),
      static_cast<int>(y + text_height / 4),
      dir.label
    );
  }
}

void HeadingIndicator::drawDegreeMarkers(QPainter & painter, double radius)
{
  const double tick_outer = radius * 0.85;
  const double major_tick_inner = radius * 0.75;  // Every 30 degrees
  const double minor_tick_inner = radius * 0.80;  // Every 10 degrees

  for (int deg = 0; deg < 360; deg += 10) {
    const double angle_rad = (deg - 90.0) * M_PI / 180.0;
    const double cos_a = std::cos(angle_rad);
    const double sin_a = std::sin(angle_rad);

    // Major ticks every 30 degrees
    if (deg % 30 == 0) {
      QPen major_pen(QColor(255, 255, 255, 230), 2.5);
      painter.setPen(major_pen);
      
      painter.drawLine(
        QPointF(cos_a * major_tick_inner, sin_a * major_tick_inner),
        QPointF(cos_a * tick_outer, sin_a * tick_outer)
      );
    }
    // Minor ticks every 10 degrees (but not on major tick positions)
    else {
      QPen minor_pen(QColor(255, 255, 255, 150), 1.5);
      painter.setPen(minor_pen);
      
      painter.drawLine(
        QPointF(cos_a * minor_tick_inner, sin_a * minor_tick_inner),
        QPointF(cos_a * tick_outer, sin_a * tick_outer)
      );
    }
  }
}

void HeadingIndicator::drawHeadingPointer(QPainter & painter, double radius)
{
  // Draw a fixed triangle pointer at the top (12 o'clock position)
  const double pointer_base = radius * 0.12;
  const double pointer_height = radius * 0.20;
  const double pointer_y = -radius * 0.95;

  QPolygonF pointer;
  pointer << QPointF(0, pointer_y)
          << QPointF(-pointer_base / 2, pointer_y + pointer_height)
          << QPointF(pointer_base / 2, pointer_y + pointer_height);

  painter.setPen(QPen(QColor(255, 200, 0), 2));  // Gold/amber color
  painter.setBrush(QColor(255, 200, 0, 200));
  painter.drawPolygon(pointer);

  // Draw a small line extending from pointer to center area
  painter.setPen(QPen(QColor(255, 200, 0, 150), 1.5));
  painter.drawLine(
    QPointF(0, pointer_y + pointer_height),
    QPointF(0, -radius * 0.70)
  );
}

void HeadingIndicator::drawHeadingText(QPainter & painter)
{
  const int width = this->width();
  const int height = this->height();

  // Format heading as integer with degree symbol
  const int heading_int = static_cast<int>(std::round(heading_));
  const QString text = QString("%1°").arg(heading_int, 3, 10, QChar('0'));

  // Set font for heading text
  QFont font;
  font.setPixelSize(std::min(width, height) / 8);
  font.setBold(true);
  painter.setFont(font);

  // Draw text with background for readability
  QFontMetrics metrics(font);
  const int text_width = metrics.horizontalAdvance(text);
  const int text_height = metrics.height();
  const int text_x = (width - text_width) / 2;
  const int text_y = height - text_height / 2;

  // Draw semi-transparent background
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(0, 0, 0, 150));
  painter.drawRoundedRect(
    text_x - 5,
    text_y - text_height + 5,
    text_width + 10,
    text_height + 5,
    5, 5
  );

  // Draw text
  painter.setPen(QPen(QColor(255, 200, 0), 2));  // Gold/amber color
  painter.drawText(text_x, text_y, text);
}

double HeadingIndicator::normalizeAngle(double angle) const
{
  // Normalize to [0, 360) range
  angle = std::fmod(angle, 360.0);
  if (angle < 0.0) {
    angle += 360.0;
  }
  return angle;
}

}  // namespace widgets
}  // namespace rviz_attitude_plugin
