#include "rviz_attitude_plugin/widgets/heading_indicator.hpp"

#include <QPainter>
#include <QLinearGradient>
#include <QConicalGradient>
#include <QRadialGradient>
#include <QBrush>
#include <QPen>
#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QPolygonF>
#include <QPointF>
#include <QRectF>
#include <QSizePolicy>
#include <cmath>
#include <algorithm>

namespace rviz_attitude_plugin
{
namespace widgets
{

HeadingIndicator::HeadingIndicator(QWidget * parent)
: QWidget(parent),
  yaw_(0.0),
  scale_factor_(1.0)
{
  setMinimumSize(60, 60);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void HeadingIndicator::setHeading(double yaw)
{
  yaw_ = std::fmod(yaw, 360.0);
  if (yaw_ < 0) {
    yaw_ += 360.0;
  }
  update();
}

QSize HeadingIndicator::sizeHint() const
{
  return QSize(160, 160);
}

void HeadingIndicator::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setRenderHint(QPainter::SmoothPixmapTransform);

  const int width = this->width();
  const int height = this->height();
  const int size = std::min(width, height);
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  const double radius = size / 2.0 - 6.0;

  scale_factor_ = size > 0 ? size / 250.0 : 1.0;

  painter.translate(cx, cy);

  draw3DCompassBezel(painter, radius);

  painter.save();
  painter.rotate(-yaw_);
  drawRotatingCompassRose(painter, radius * 0.75);
  painter.restore();

  drawFixedOuterRing(painter, radius);
}

void HeadingIndicator::draw3DCompassBezel(QPainter & painter, double radius)
{
  // Shadow rings
  for (int i = 0; i < 5; ++i) {
    const int opacity = 50 - i * 10;
    painter.setPen(QPen(QColor(0, 0, 0, opacity), 1));
    painter.drawEllipse(QPointF(0, 0), radius + i, radius + i);
  }

  // 3D rim gradient
  QConicalGradient rim_gradient(0, 0, 0);
  rim_gradient.setColorAt(0.00, QColor(100, 100, 110));
  rim_gradient.setColorAt(0.25, QColor(140, 140, 150));
  rim_gradient.setColorAt(0.50, QColor(100, 100, 110));
  rim_gradient.setColorAt(0.75, QColor(60, 60, 70));
  rim_gradient.setColorAt(1.00, QColor(100, 100, 110));

  painter.setPen(QPen(QColor(80, 80, 90), 2));
  painter.setBrush(QBrush(rim_gradient));
  painter.drawEllipse(QPointF(0, 0), radius, radius);

  // Background gradient
  QRadialGradient bg_gradient(0, 0, radius - 5);
  bg_gradient.setColorAt(0.0, QColor(40, 40, 45));
  bg_gradient.setColorAt(0.7, QColor(25, 25, 30));
  bg_gradient.setColorAt(1.0, QColor(15, 15, 20));

  painter.setPen(Qt::NoPen);
  painter.setBrush(QBrush(bg_gradient));
  painter.drawEllipse(QPointF(0, 0), radius - 5, radius - 5);
}

void HeadingIndicator::drawFixedOuterRing(QPainter & painter, double radius)
{
  const double sf = scale_factor_;
  const double major_tick_len = std::max(12.0, 15.0 * sf);
  const double minor_tick_len = std::max(7.0, 10.0 * sf);
  const double ring_inset = std::max(3.0, 3.0 * sf);
  const double label_pad = std::max(4.0, 6.0 * sf);
  const double deg_pad = std::max(6.0, 8.0 * sf);

  const double cardinal_r = radius - ring_inset - major_tick_len - label_pad;
  const double degree_r = radius - ring_inset - major_tick_len - deg_pad;

  // Cardinal directions
  const int cardinal_font_size = std::max(8, static_cast<int>(12 * sf));
  painter.setFont(QFont("Arial", cardinal_font_size, QFont::Bold));
  const QFontMetrics fm_card = painter.fontMetrics();

  const std::map<int, QString> cardinal_directions = {
    {0, "E"}, {90, "S"}, {180, "W"}, {270, "N"}
  };

  for (int angle = 0; angle < 360; angle += 30) {
    // Major tick every 30°
    painter.save();
    painter.rotate(angle);
    painter.setPen(QPen(QColor(180, 180, 180), 2));
    painter.drawLine(
      QPointF(0, -radius + ring_inset),
      QPointF(0, -radius + ring_inset + major_tick_len));
    painter.restore();

    // Cardinal letters at 0/90/180/270
    if (cardinal_directions.count(angle)) {
      const QString text = cardinal_directions.at(angle);
      const int text_w = std::max(20, static_cast<int>(30 * sf));
      const int text_h = fm_card.height();

      painter.save();
      painter.rotate(angle);
      painter.translate(0, -cardinal_r + 18);
      painter.rotate(-angle);
      painter.setPen(QPen(QColor(255, 255, 255), 1));
      painter.drawText(
        QRectF(-text_w / 2.0, -text_h / 2.0, text_w, text_h),
        Qt::AlignCenter,
        text);
      painter.restore();
    }
  }

  // Degree numbers
  const int degree_font_size = std::max(6, static_cast<int>(8 * sf));
  painter.setFont(QFont("Arial", degree_font_size, QFont::Normal));
  const QFontMetrics fm_deg = painter.fontMetrics();

  for (int angle = 0; angle < 360; angle += 30) {
    int display_angle = angle > 180 ? angle - 360 : angle;
    const QString text = (display_angle == 180 || display_angle == -180)
      ? QString("±180")
      : QString::number(-display_angle);
    const int text_w = std::max(20, static_cast<int>(30 * sf));
    const int text_h = fm_deg.height();

    painter.save();
    painter.rotate(angle);
    painter.setPen(QPen(QColor(160, 160, 160), 1));
    painter.translate(0, -degree_r + 2);
    painter.rotate(-angle);
    painter.drawText(
      QRectF(-text_w / 2.0, -text_h / 2.0, text_w, text_h),
      Qt::AlignCenter,
      text);
    painter.restore();
  }

  // Minor ticks every 10° (except at 30° multiples)
  for (int angle = 0; angle < 360; angle += 10) {
    if (angle % 30 != 0) {
      painter.save();
      painter.rotate(angle);
      painter.setPen(QPen(QColor(120, 120, 120), 1.5));
      painter.drawLine(
        QPointF(0, -radius + ring_inset),
        QPointF(0, -radius + ring_inset + minor_tick_len));
      painter.restore();
    }
  }
}

void HeadingIndicator::drawRotatingCompassRose(QPainter & painter, double radius)
{
  painter.save();

  const double chevron_height = radius * 0.45;
  const double chevron_width = radius * 0.55;
  const double tip_y = -radius * 0.6;

  // Heading pointer chevron
  QPolygonF chevron;
  chevron << QPointF(0, tip_y)
          << QPointF(-chevron_width * 0.5, chevron_height * 0.3)
          << QPointF(-chevron_width * 0.25, chevron_height * 0.6)
          << QPointF(0, chevron_height * 0.4)
          << QPointF(chevron_width * 0.25, chevron_height * 0.6)
          << QPointF(chevron_width * 0.5, chevron_height * 0.3);

  QLinearGradient gradient(0, tip_y, 0, chevron_height);
  gradient.setColorAt(0.0, QColor(255, 90, 90));
  gradient.setColorAt(0.3, QColor(240, 40, 40));
  gradient.setColorAt(0.7, QColor(180, 10, 10));
  gradient.setColorAt(1.0, QColor(120, 0, 0));

  painter.setBrush(QBrush(gradient));
  painter.setPen(QPen(QColor(60, 0, 0), 2));
  painter.drawPolygon(chevron);

  // Glow effect
  for (int i = 0; i < 4; ++i) {
    const double glow_scale = 1.0 + i * 0.03;
    const int alpha = 120 - i * 30;
    QPolygonF glow;
    for (const QPointF & pt : chevron) {
      glow << pt * glow_scale;
    }
    painter.setPen(QPen(QColor(255, 50, 50, alpha), 1));
    painter.setBrush(Qt::NoBrush);
    painter.drawPolygon(glow);
  }

  // Center hub
  const double hub_radius = radius * 0.08;
  QRadialGradient hub_gradient(0, 0, hub_radius);
  hub_gradient.setColorAt(0.0, QColor(250, 250, 255));
  hub_gradient.setColorAt(0.4, QColor(100, 100, 120));
  hub_gradient.setColorAt(1.0, QColor(40, 40, 50));

  painter.setBrush(QBrush(hub_gradient));
  painter.setPen(QPen(QColor(180, 180, 200), 1));
  painter.drawEllipse(QPointF(0, 0), hub_radius, hub_radius);

  painter.restore();
}

}  // namespace widgets
}  // namespace rviz_attitude_plugin
