/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Angle Readout Implementation
 */

#include "rviz_attitude_plugin/widgets/angle_readout.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QPen>
#include <QBrush>
#include <QFont>
#include <QFontMetrics>
#include <QRectF>
#include <cmath>

namespace rviz_attitude_plugin
{
namespace widgets
{

AngleReadout::AngleReadout(QWidget * parent)
: QWidget(parent),
  value_(0.0),
  title_("ANGLE"),
  color_(Qt::white),
  background_color_(QColor(40, 40, 60))
{
  setMinimumSize(120, 80);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}

void AngleReadout::setValue(double value)
{
  value_ = value;
  update();
}

void AngleReadout::setTitle(const QString & title)
{
  title_ = title;
  update();
}

void AngleReadout::setColor(const QColor & color)
{
  color_ = color;
  update();
}

void AngleReadout::setBackgroundColor(const QColor & color)
{
  background_color_ = color;
  update();
}

QSize AngleReadout::sizeHint() const
{
  return QSize(140, 90);
}

QSize AngleReadout::minimumSizeHint() const
{
  return QSize(100, 70);
}

void AngleReadout::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const int width = this->width();
  const int height = this->height();
  const double margin = 5.0;

  // Main content rectangle
  QRectF content_rect(margin, margin, width - 2 * margin, height - 2 * margin);

  // Draw glass effect background
  drawGlassBackground(painter, content_rect);

  // Draw shadow effects for depth
  drawShadowEffects(painter, content_rect);

  // Split into title and value areas
  const double title_height = content_rect.height() * 0.35;
  QRectF title_rect(content_rect.left(), content_rect.top(), 
                    content_rect.width(), title_height);
  QRectF value_rect(content_rect.left(), content_rect.top() + title_height,
                    content_rect.width(), content_rect.height() - title_height);

  // Draw title
  drawTitle(painter, title_rect);

  // Draw value
  drawValue(painter, value_rect);
}

void AngleReadout::drawGlassBackground(QPainter & painter, const QRectF & rect)
{
  // Rounded rectangle path
  QPainterPath path;
  const double corner_radius = 8.0;
  path.addRoundedRect(rect, corner_radius, corner_radius);

  // Base gradient (darker at bottom, lighter at top for glass effect)
  QLinearGradient base_gradient(rect.topLeft(), rect.bottomLeft());
  
  QColor top_color = background_color_.lighter(130);
  QColor mid_color = background_color_;
  QColor bottom_color = background_color_.darker(120);
  
  base_gradient.setColorAt(0.0, top_color);
  base_gradient.setColorAt(0.4, mid_color);
  base_gradient.setColorAt(1.0, bottom_color);

  painter.fillPath(path, base_gradient);

  // Glass shine effect (top highlight)
  QLinearGradient shine_gradient(
    QPointF(rect.center().x(), rect.top()),
    QPointF(rect.center().x(), rect.top() + rect.height() * 0.4)
  );
  shine_gradient.setColorAt(0.0, QColor(255, 255, 255, 60));
  shine_gradient.setColorAt(0.5, QColor(255, 255, 255, 20));
  shine_gradient.setColorAt(1.0, QColor(255, 255, 255, 0));

  QPainterPath shine_path;
  QRectF shine_rect = rect.adjusted(2, 2, -2, -rect.height() * 0.5);
  shine_path.addRoundedRect(shine_rect, corner_radius - 2, corner_radius - 2);
  painter.fillPath(shine_path, shine_gradient);

  // Border for definition
  QPen border_pen(QColor(255, 255, 255, 100), 1.5);
  painter.setPen(border_pen);
  painter.setBrush(Qt::NoBrush);
  painter.drawPath(path);
}

void AngleReadout::drawShadowEffects(QPainter & painter, const QRectF & rect)
{
  // Inner shadow at top (creates depth)
  QLinearGradient inner_shadow(rect.topLeft(), 
                                QPointF(rect.left(), rect.top() + 15));
  inner_shadow.setColorAt(0.0, QColor(0, 0, 0, 80));
  inner_shadow.setColorAt(1.0, QColor(0, 0, 0, 0));

  QPainterPath shadow_path;
  shadow_path.addRoundedRect(rect.adjusted(2, 2, -2, -rect.height() + 15), 6, 6);
  painter.fillPath(shadow_path, inner_shadow);

  // Subtle highlight at bottom edge
  painter.setPen(QPen(QColor(255, 255, 255, 30), 1));
  painter.drawLine(
    QPointF(rect.left() + 10, rect.bottom() - 2),
    QPointF(rect.right() - 10, rect.bottom() - 2)
  );
}

void AngleReadout::drawTitle(QPainter & painter, const QRectF & rect)
{
  // Title font (smaller, uppercase)
  QFont title_font;
  title_font.setPixelSize(static_cast<int>(rect.height() * 0.45));
  title_font.setBold(true);
  title_font.setLetterSpacing(QFont::AbsoluteSpacing, 1.5);
  painter.setFont(title_font);

  // Draw title with slight shadow for depth
  // Shadow
  painter.setPen(QPen(QColor(0, 0, 0, 150), 1));
  painter.drawText(
    rect.adjusted(1, 1, 1, 1),
    Qt::AlignCenter,
    title_.toUpper()
  );

  // Main text
  QColor title_color = color_;
  title_color.setAlpha(200);
  painter.setPen(QPen(title_color, 1));
  painter.drawText(rect, Qt::AlignCenter, title_.toUpper());
}

void AngleReadout::drawValue(QPainter & painter, const QRectF & rect)
{
  // Format value with degree symbol
  QString value_text = QString("%1°").arg(value_, 0, 'f', 1);

  // Value font (larger, bold)
  QFont value_font;
  value_font.setPixelSize(static_cast<int>(rect.height() * 0.55));
  value_font.setBold(true);
  painter.setFont(value_font);

  // Draw value with shadow for depth
  // Shadow
  painter.setPen(QPen(QColor(0, 0, 0, 180), 2));
  painter.drawText(
    rect.adjusted(2, 2, 2, 2),
    Qt::AlignCenter,
    value_text
  );

  // Main text with slight glow effect
  painter.setPen(QPen(color_, 2));
  painter.drawText(rect, Qt::AlignCenter, value_text);

  // Glow/highlight effect
  QColor glow_color = color_.lighter(150);
  glow_color.setAlpha(100);
  painter.setPen(QPen(glow_color, 1));
  painter.drawText(
    rect.adjusted(0, -1, 0, -1),
    Qt::AlignCenter,
    value_text
  );
}

}  // namespace widgets
}  // namespace rviz_attitude_plugin
