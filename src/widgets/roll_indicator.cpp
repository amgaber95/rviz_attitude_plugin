/*
 * RViz Attitude Display Plugin - Roll Indicator Implementation
 */

#include "rviz_attitude_plugin/widgets/roll_indicator.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QPointF>
#include <QPolygonF>
#include <cmath>
#include <algorithm>

namespace rviz_attitude_plugin
{
namespace widgets
{

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

}  // namespace widgets
}  // namespace rviz_attitude_plugin
