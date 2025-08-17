/*
 * RViz Attitude Display Plugin - Roll Indicator Widget
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__ROLL_INDICATOR_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__ROLL_INDICATOR_HPP_

#include <QWidget>

namespace rviz_attitude_plugin
{
namespace widgets
{

/**
 * @brief Roll indicator arc showing roll angle scale.
 * 
 * Displays the roll scale around the perimeter of the attitude indicator:
 * - Arc with tick marks at major angles (30°, 45°, 60°)
 * - Minor tick marks at 10° intervals
 * - Rotating pointer triangle indicating current roll angle
 */
class RollIndicator : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(double rollAngle READ rollAngle WRITE setRollAngle)

public:
  explicit RollIndicator(QWidget * parent = nullptr);
  ~RollIndicator() override = default;

  // Getters
  double rollAngle() const { return roll_; }

  // Setters
  void setRollAngle(double roll);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  double roll_;  // degrees, positive = right wing down
};

}  // namespace widgets
}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__WIDGETS__ROLL_INDICATOR_HPP_
