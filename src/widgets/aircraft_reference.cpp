/*
 * RViz Attitude Display Plugin - Aircraft Reference Implementation
 */

#include "rviz_attitude_plugin/widgets/aircraft_reference.hpp"

#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QPointF>
#include <QLineF>

namespace rviz_attitude_plugin
{
namespace widgets
{

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

}  // namespace widgets
}  // namespace rviz_attitude_plugin
