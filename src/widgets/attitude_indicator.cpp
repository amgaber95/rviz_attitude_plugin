/*
 * RViz Attitude Display Plugin - Attitude Indicator Implementation
 */

#include "rviz_attitude_plugin/widgets/attitude_indicator.hpp"
#include "rviz_attitude_plugin/widgets/artificial_horizon.hpp"
#include "rviz_attitude_plugin/widgets/aircraft_reference.hpp"
#include "rviz_attitude_plugin/widgets/roll_indicator.hpp"

#include <QVBoxLayout>
#include <QResizeEvent>

namespace rviz_attitude_plugin
{
namespace widgets
{

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
