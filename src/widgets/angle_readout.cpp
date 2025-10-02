#include "rviz_attitude_plugin/widgets/angle_readout.hpp"

#include <QPainter>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QBrush>
#include <QPen>
#include <QColor>
#include <QFont>
#include <QRectF>
#include <QPointF>
#include <QSizePolicy>
#include <algorithm>

namespace rviz_attitude_plugin
{
namespace widgets
{

AngleReadout::AngleReadout(
  const QString & title,
  const QString & color,
  QWidget * parent)
: QWidget(parent),
  color_(color),
  title_(title),
  value_("0.0"),
  r_(59), g_(130), b_(246)
{
  setObjectName("AngleReadout");
  parseColor();
  setMinimumSize(40, 24);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void AngleReadout::setValue(const QString & text)
{
  value_ = text;
  update();
}

void AngleReadout::setColor(const QString & color)
{
  color_ = color;
  parseColor();
  update();
}

QSize AngleReadout::minimumSizeHint() const
{
  return QSize(60, 40);
}

void AngleReadout::parseColor()
{
  QString color_str = color_;
  if (color_str.startsWith('#')) {
    color_str = color_str.mid(1);
  }

  if (color_str.length() == 6) {
    bool ok;
    r_ = color_str.mid(0, 2).toInt(&ok, 16);
    g_ = color_str.mid(2, 2).toInt(&ok, 16);
    b_ = color_str.mid(4, 2).toInt(&ok, 16);
  } else {
    r_ = 59;
    g_ = 130;
    b_ = 246;
  }
}

void AngleReadout::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setRenderHint(QPainter::TextAntialiasing);

  const int width = this->width();
  const int height = this->height();

  if (width <= 0 || height <= 0) {
    return;
  }

  // Scaling factor tuned for compact rendering
  const double scale = std::min(width / 120.0, height / 70.0);

  // Title
  const double title_height = height * 0.30;
  const int title_font_size = std::max(10, static_cast<int>(scale * 9));
  painter.setPen(QPen(QColor(160, 165, 185)));
  painter.setFont(QFont("Cascadia Code", title_font_size, QFont::Bold));
  const QRectF title_rect(0, height * 0.04, width, title_height);
  painter.drawText(title_rect, Qt::AlignCenter, title_.toUpper());

  // Display box dimensions
  const double box_top = title_height + height * 0.04;
  const double margin_x = width * 0.04;
  const double margin_y = height * 0.03;
  const QRectF box_rect(margin_x, box_top, width - 2 * margin_x, height - box_top - margin_y);
  const double corner_radius = std::max(2.0, scale * 6);

  // Simple shadow
  const double shadow_offset = scale * 2;
  painter.setPen(Qt::NoPen);
  painter.setBrush(QBrush(QColor(0, 0, 0, 60)));
  painter.drawRoundedRect(
    box_rect.adjusted(shadow_offset, shadow_offset, shadow_offset, shadow_offset),
    corner_radius, corner_radius);

  // Bezel gradient
  QLinearGradient bezel_gradient(0, box_rect.top(), 0, box_rect.bottom());
  bezel_gradient.setColorAt(0.0, QColor(55, 58, 70));
  bezel_gradient.setColorAt(0.5, QColor(40, 43, 52));
  bezel_gradient.setColorAt(1.0, QColor(30, 33, 42));
  painter.setBrush(QBrush(bezel_gradient));
  painter.setPen(QPen(QColor(65, 70, 85), std::max(1, static_cast<int>(scale * 1.5))));
  painter.drawRoundedRect(box_rect, corner_radius, corner_radius);

  // Inner screen
  const double inset = std::max(2.0, scale * 2.5);
  const QRectF inner_rect = box_rect.adjusted(inset, inset, -inset, -inset);
  const double inner_corner = corner_radius * 0.7;

  QLinearGradient screen_gradient(0, inner_rect.top(), 0, inner_rect.bottom());
  screen_gradient.setColorAt(0.0, QColor(18, 20, 26));
  screen_gradient.setColorAt(0.5, QColor(24, 26, 34));
  screen_gradient.setColorAt(1.0, QColor(16, 18, 24));
  painter.setBrush(QBrush(screen_gradient));
  painter.setPen(QPen(QColor(10, 12, 16)));
  painter.drawRoundedRect(inner_rect, inner_corner, inner_corner);

  // Glass highlight
  painter.setClipRect(inner_rect);
  const double highlight_height = inner_rect.height() * 0.22;
  QLinearGradient glass_gradient(0, inner_rect.top(), 0, inner_rect.top() + highlight_height);
  glass_gradient.setColorAt(0.0, QColor(255, 255, 255, 30));
  glass_gradient.setColorAt(1.0, QColor(255, 255, 255, 0));
  painter.setBrush(QBrush(glass_gradient));
  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(
    inner_rect.adjusted(0, 0, 0, -inner_rect.height() / 2),
    inner_corner, inner_corner);
  painter.setClipping(false);

  // Glow behind text
  const QPointF glow_center = inner_rect.center();
  const double glow_radius = std::min(inner_rect.width(), inner_rect.height()) * 0.4;
  QRadialGradient glow_gradient(glow_center, glow_radius);
  glow_gradient.setColorAt(0.0, QColor(r_, g_, b_, 50));
  glow_gradient.setColorAt(0.5, QColor(r_, g_, b_, 20));
  glow_gradient.setColorAt(1.0, QColor(r_, g_, b_, 0));
  painter.setBrush(QBrush(glow_gradient));
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(glow_center, glow_radius, glow_radius * 0.7);

  // Value text
  const int value_font_size = std::max(10, static_cast<int>(scale * 20));
  const QFont value_font("Consolas", value_font_size, QFont::Bold);
  painter.setFont(value_font);

  // Text shadow
  const double text_shadow_offset = std::max(1.0, scale * 1.0);
  painter.setPen(QPen(QColor(0, 0, 0, 100)));
  const QRectF shadow_rect(
    inner_rect.left() + text_shadow_offset,
    inner_rect.top() + text_shadow_offset,
    inner_rect.width(),
    inner_rect.height());
  painter.drawText(shadow_rect, Qt::AlignCenter, value_);

  // Main text with subtle glow
  painter.setPen(QPen(QColor(r_, g_, b_, 30), std::max(1.0, scale * 1.5)));
  painter.drawText(inner_rect, Qt::AlignCenter, value_);

  painter.setPen(QPen(QColor(r_, g_, b_).lighter(110)));
  painter.drawText(inner_rect, Qt::AlignCenter, value_);
}

}  // namespace widgets
}  // namespace rviz_attitude_plugin
