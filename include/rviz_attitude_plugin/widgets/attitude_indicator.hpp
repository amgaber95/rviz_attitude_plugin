/*
 * RViz Attitude Display Plugin - Attitude Indicator Composite Widget
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__ATTITUDE_INDICATOR_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__ATTITUDE_INDICATOR_HPP_

#include <QWidget>

namespace rviz_attitude_plugin
{
namespace widgets
{

// Forward declarations
class ArtificialHorizon;
class AircraftReference;
class RollIndicator;

/**
 * @brief Composite attitude indicator combining all components.
 * 
 * Uses the composite pattern to manage and coordinate:
 * - ArtificialHorizon: Sky/ground gradient background
 * - AircraftReference: Fixed center crosshair symbol
 * - RollIndicator: Roll scale arc with pointer
 * 
 * All components are stacked and properly layered to create
 * the complete attitude indicator display.
 */
class AttitudeIndicator : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(double pitchAngle READ pitchAngle WRITE setPitchAngle)
  Q_PROPERTY(double rollAngle READ rollAngle WRITE setRollAngle)
  Q_PROPERTY(bool showRollIndicator READ showRollIndicator WRITE setShowRollIndicator)

public:
  explicit AttitudeIndicator(QWidget * parent = nullptr);
  ~AttitudeIndicator() override = default;

  // Getters
  double pitchAngle() const { return pitch_; }
  double rollAngle() const { return roll_; }
  bool showRollIndicator() const { return show_roll_indicator_; }

  // Setters
  void setPitchAngle(double pitch);
  void setRollAngle(double roll);
  void setAttitude(double pitch, double roll);
  void setShowRollIndicator(bool show);

  QSize sizeHint() const override;

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
  void updateChildGeometry();

  // Child components
  ArtificialHorizon * horizon_;
  AircraftReference * aircraft_;
  RollIndicator * roll_indicator_;

  // State
  double pitch_;
  double roll_;
  bool show_roll_indicator_;
};

}  // namespace widgets
}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__WIDGETS__ATTITUDE_INDICATOR_HPP_
