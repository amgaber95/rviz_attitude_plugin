

#include "rviz_attitude_plugin/widgets/attitude_indicator.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QLinearGradient>
#include <QBrush>
#include <QPen>
#include <QColor>
#include <QFont>
#include <QPolygonF>
#include <QPointF>
#include <QRectF>
#include <QResizeEvent>
#include <QSizePolicy>
#include <cmath>
#include <algorithm>
#include <vector>

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
  setMinimumSize(60, 60);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setAttribute(Qt::WA_TranslucentBackground);
}

void ArtificialHorizon::setPitchAngle(double pitch)
{
  pitch_ = std::clamp(pitch, -90.0, 90.0);
  update();
}

void ArtificialHorizon::setRollAngle(double roll)
{
  roll_ = roll;
  update();
}

void ArtificialHorizon::setAttitude(double pitch, double roll)
{
  pitch_ = std::clamp(pitch, -90.0, 90.0);
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
  return QSize(160, 160);
}

void ArtificialHorizon::paintEvent(QPaintEvent * /*event*/)
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

  if (radius <= 0) {
    return;
  }

  painter.translate(cx, cy);

  // Clip to circular bezel
  QPainterPath clip_path;
  clip_path.addEllipse(QPointF(0, 0), radius, radius);
  painter.setClipPath(clip_path);

  // Apply opacity if needed
  if (background_opacity_ < 1.0) {
    painter.setOpacity(background_opacity_);
  }

  painter.save();
  painter.rotate(roll_);
  if (background_visible_) {
    drawSkyGround(painter, radius);
  }
  painter.restore();

  painter.setClipping(false);
  drawOuterRing(painter, radius);
}

void ArtificialHorizon::drawSkyGround(QPainter & painter, double radius)
{
  const double px_per_deg = radius / 30.0;  // 30 degrees visible range
  const double pitch_offset = -pitch_ * px_per_deg;

  // Sky gradient
  QLinearGradient sky_gradient(0, -radius, 0, pitch_offset);
  sky_gradient.setColorAt(0.0, QColor(0, 80, 160));
  sky_gradient.setColorAt(0.3, QColor(0, 120, 200));
  sky_gradient.setColorAt(0.7, QColor(30, 150, 220));
  sky_gradient.setColorAt(1.0, QColor(135, 206, 250));

  // Ground gradient
  QLinearGradient ground_gradient(0, pitch_offset, 0, radius);
  ground_gradient.setColorAt(0.0, QColor(85, 140, 85));
  ground_gradient.setColorAt(0.3, QColor(65, 120, 65));
  ground_gradient.setColorAt(0.7, QColor(45, 100, 45));
  ground_gradient.setColorAt(1.0, QColor(25, 80, 25));

  painter.fillRect(
    QRectF(-radius, -radius, radius * 2, pitch_offset + radius),
    QBrush(sky_gradient));

  painter.fillRect(
    QRectF(-radius, pitch_offset, radius * 2, radius - pitch_offset),
    QBrush(ground_gradient));

  // Horizon line with glow
  painter.setPen(QPen(QColor(255, 255, 255, 60), 6));
  painter.drawLine(QPointF(-radius, pitch_offset), QPointF(radius, pitch_offset));
  painter.setPen(QPen(QColor(255, 255, 255), 3));
  painter.drawLine(QPointF(-radius, pitch_offset), QPointF(radius, pitch_offset));
  painter.setPen(QPen(QColor(255, 255, 100), 1));
  painter.drawLine(QPointF(-radius, pitch_offset), QPointF(radius, pitch_offset));
}

void ArtificialHorizon::drawOuterRing(QPainter & painter, double radius)
{
  painter.setPen(QPen(QColor(80, 80, 80), 2));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(QPointF(0, 0), radius, radius);
}

PitchLadder::PitchLadder(QWidget * parent)
: QWidget(parent),
  pitch_(0.0),
  roll_(0.0),
  ladder_range_(90.0),
  ladder_step_(10.0)
{
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setAttribute(Qt::WA_TranslucentBackground);
  setAutoFillBackground(false);
}

void PitchLadder::setPitchAngle(double pitch)
{
  pitch_ = std::clamp(pitch, -90.0, 90.0);
  update();
}

void PitchLadder::setRollAngle(double roll)
{
  roll_ = roll;
  update();
}

void PitchLadder::setLadderRange(double max_degrees)
{
  ladder_range_ = std::clamp(max_degrees, 30.0, 90.0);
  update();
}

void PitchLadder::setLadderStep(double step)
{
  ladder_step_ = std::clamp(step, 5.0, 20.0);
  update();
}

void PitchLadder::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const int width = this->width();
  const int height = this->height();
  const int size = std::min(width, height);
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  const double radius = size / 2.0 - 6.0;

  if (radius <= 0) {
    return;
  }

  // Translate to center and rotate for roll
  painter.translate(cx, cy);
  painter.rotate(roll_);

  // Clip to circular area
  QPainterPath clip_path;
  clip_path.addEllipse(QPointF(0, 0), radius, radius);
  painter.setClipPath(clip_path);

  const double px_per_deg = radius / 30.0;  // 30 degrees visible range
  painter.setFont(QFont("Arial", 8, QFont::Bold));

  // Draw ladder lines
  for (int angle = -static_cast<int>(ladder_range_); 
       angle <= static_cast<int>(ladder_range_); 
       angle += static_cast<int>(ladder_step_)) {
    if (angle == 0) {
      continue;  // Skip horizon line (drawn by ArtificialHorizon)
    }

    const double y = -angle * px_per_deg - pitch_ * px_per_deg;
    if (std::abs(y) > radius) {
      continue;
    }

    // Determine line length and width based on angle
    double length = (angle % 30 == 0) ? 40.0 : 25.0;
    double pen_width = (angle % 30 == 0) ? 2.5 : 2.0;

    painter.setPen(QPen(QColor(255, 255, 255), pen_width));

    if (angle > 0) {
      // Positive pitch - broken lines
      painter.drawLine(QPointF(-length, y), QPointF(-5, y));
      painter.drawLine(QPointF(5, y), QPointF(length, y));
    } else {
      // Negative pitch - solid lines
      painter.drawLine(QPointF(-length, y), QPointF(length, y));
    }

    // Degree labels
    painter.setPen(QPen(QColor(255, 255, 255), 1));
    painter.drawText(
      QRectF(-length - 25, y - 10, 20, 20),
      Qt::AlignRight | Qt::AlignVCenter,
      QString::number(std::abs(angle)));
    painter.drawText(
      QRectF(length + 5, y - 10, 20, 20),
      Qt::AlignLeft | Qt::AlignVCenter,
      QString::number(std::abs(angle)));
  }
}

// ============================================================================
// AircraftReference Implementation
// ============================================================================

AircraftReference::AircraftReference(QWidget * parent)
: QWidget(parent),
  color_(255, 200, 0)  // Yellow/amber color
{
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setAttribute(Qt::WA_TranslucentBackground);
  setAutoFillBackground(false);
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
  const int size = std::min(width, height);
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  const double radius = size / 2.0 - 6.0;

  if (radius <= 0) {
    return;
  }

  painter.translate(cx, cy);

  // Draw center dot
  painter.setPen(QPen(color_, 1));
  painter.setBrush(QBrush(color_));
  painter.drawEllipse(QPointF(0, 0), 4, 4);

  // Draw wing indicators
  const double wing_length = radius * 0.4;
  painter.setPen(QPen(color_, 3));
  
  // Left wing
  painter.drawLine(QPointF(-10, 0), QPointF(-wing_length, 0));
  painter.drawLine(QPointF(-wing_length, -10), QPointF(-wing_length, 0));
  
  // Right wing
  painter.drawLine(QPointF(10, 0), QPointF(wing_length, 0));
  painter.drawLine(QPointF(wing_length, -10), QPointF(wing_length, 0));
}

// ============================================================================
// RollIndicator Implementation
// ============================================================================

RollIndicator::RollIndicator(QWidget * parent)
: QWidget(parent),
  roll_(0.0),
  scale_factor_(1.0)
{
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setAttribute(Qt::WA_TranslucentBackground);
  setAutoFillBackground(false);
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
  const int size = std::min(width, height);
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  const double radius = size / 2.0 - 6.0;

  if (radius <= 0) {
    return;
  }

  scale_factor_ = size > 0 ? size / 250.0 : 1.0;

  painter.translate(cx, cy);
  painter.save();

  // // Draw tick marks
  // const std::vector<int> angles = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};

  // for (int angle : angles) {
  //   painter.save();
  //   painter.rotate(angle);

  //   double length, width;
  //   QColor color;
  //   bool show_number;

  //   if (angle == 0) {
  //     length = 15;
  //     width = 3;
  //     color = QColor(255, 200, 0);
  //     show_number = true;
  //   } else if (angle % 30 == 0) {
  //     length = 12;
  //     width = 2;
  //     color = QColor(255, 255, 255);
  //     show_number = true;
  //   } else {
  //     length = (angle % 10 == 0) ? 10 : 8;
  //     width = 2;
  //     color = QColor(255, 255, 255);
  //     show_number = false;
  //   }

  //   painter.setPen(QPen(color, width));
  //   painter.drawLine(QPointF(0, -radius + 5), QPointF(0, -radius + 5 + length));

  //   if (show_number) {
  //     const int font_size = std::max(6, static_cast<int>(8 * scale_factor_));
  //     painter.setFont(QFont("Arial", font_size, QFont::Bold));
  //     const double text_y = -radius - std::max(12.0, 15.0 * scale_factor_);

  //     // Shadow
  //     painter.setPen(QPen(QColor(0, 0, 0, 180), 1));
  //     painter.save();
  //     painter.translate(0, text_y + 1);
  //     painter.rotate(-angle);
  //     const int text_rect_size = std::max(20, static_cast<int>(30 * scale_factor_));
  //     painter.drawText(
  //       QRectF(-text_rect_size / 2.0, -8, text_rect_size, 16),
  //       Qt::AlignCenter,
  //       QString::number(std::abs(angle)));
  //     painter.restore();

  //     // Main text
  //     painter.setPen(QPen(color, 1));
  //     painter.save();
  //     painter.translate(0, text_y);
  //     painter.rotate(-angle);
  //     painter.drawText(
  //       QRectF(-text_rect_size / 2.0, -8, text_rect_size, 16),
  //       Qt::AlignCenter,
  //       QString::number(std::abs(angle)));
  //     painter.restore();
  //   }

  //   painter.restore();
  // }

  painter.restore();

  // Draw roll pointer triangle (rotates with roll)
  painter.save();
  painter.rotate(roll_);
  painter.setPen(QPen(QColor(255, 200, 0), 2));
  painter.setBrush(QBrush(QColor(255, 200, 0)));
  QPolygonF triangle;
  triangle << QPointF(0, -radius + 3)
           << QPointF(-8, -radius + 15)
           << QPointF(8, -radius + 15);
  painter.drawPolygon(triangle);
  painter.restore();
}

// ============================================================================
// AttitudeIndicator Implementation
// ============================================================================

AttitudeIndicator::AttitudeIndicator(QWidget * parent)
: QWidget(parent),
  show_pitch_ladder_(true),
  show_roll_indicator_(false),
  show_aircraft_ref_(true),
  pitch_ladder_range_(90.0),
  pitch_ladder_step_(10.0),
  background_visible_(true),
  background_opacity_(1.0)
{
  setMinimumSize(60, 60);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  
  setupComponents();
}

void AttitudeIndicator::setupComponents()
{
  // Create all components
  horizon_ = new ArtificialHorizon(this);
  pitch_ladder_ = new PitchLadder(this);
  aircraft_ref_ = new AircraftReference(this);
  roll_indicator_ = new RollIndicator(this);

  // Stack them in the correct order (bottom to top)
  horizon_->setGeometry(rect());
  pitch_ladder_->setGeometry(rect());
  aircraft_ref_->setGeometry(rect());
  roll_indicator_->setGeometry(rect());

  // Raise components in correct stacking order
  horizon_->lower();
  pitch_ladder_->raise();
  aircraft_ref_->raise();
  roll_indicator_->raise();

  // Apply initial visibility settings
  pitch_ladder_->setVisible(show_pitch_ladder_);
  aircraft_ref_->setVisible(show_aircraft_ref_);
  roll_indicator_->setVisible(show_roll_indicator_);
}

void AttitudeIndicator::setAttitude(double pitch, double roll)
{
  horizon_->setAttitude(pitch, roll);
  pitch_ladder_->setPitchAngle(pitch);
  pitch_ladder_->setRollAngle(roll);
  roll_indicator_->setRollAngle(roll);
}

void AttitudeIndicator::setShowPitchLadder(bool show)
{
  show_pitch_ladder_ = show;
  pitch_ladder_->setVisible(show);
}

void AttitudeIndicator::setShowRollIndicator(bool show)
{
  show_roll_indicator_ = show;
  roll_indicator_->setVisible(show);
}

void AttitudeIndicator::setShowAircraftReference(bool show)
{
  show_aircraft_ref_ = show;
  aircraft_ref_->setVisible(show);
}

void AttitudeIndicator::setPitchLadderRange(double max_degrees)
{
  pitch_ladder_range_ = max_degrees;
  pitch_ladder_->setLadderRange(max_degrees);
}

void AttitudeIndicator::setPitchLadderStep(double step)
{
  pitch_ladder_step_ = step;
  pitch_ladder_->setLadderStep(step);
}

void AttitudeIndicator::setBackgroundVisible(bool visible)
{
  background_visible_ = visible;
  horizon_->setBackgroundVisible(visible);
}

void AttitudeIndicator::setBackgroundOpacity(double opacity)
{
  background_opacity_ = opacity;
  horizon_->setBackgroundOpacity(opacity);
}

QSize AttitudeIndicator::sizeHint() const
{
  return QSize(160, 160);
}

void AttitudeIndicator::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  updateComponentGeometry();
}

void AttitudeIndicator::updateComponentGeometry()
{
  QRect r = rect();
  horizon_->setGeometry(r);
  pitch_ladder_->setGeometry(r);
  aircraft_ref_->setGeometry(r);
  roll_indicator_->setGeometry(r);
}

}  // namespace widgets
}  // namespace rviz_attitude_plugin
