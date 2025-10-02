/*
 * RViz Attitude Display Plugin - Attitude Indicator Components
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__ATTITUDE_INDICATOR_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__ATTITUDE_INDICATOR_HPP_

#include <QWidget>
#include <QColor>

namespace rviz_attitude_plugin
{
namespace widgets
{

class ArtificialHorizon : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(double pitchAngle READ pitchAngle WRITE setPitchAngle)
  Q_PROPERTY(double rollAngle READ rollAngle WRITE setRollAngle)
  Q_PROPERTY(bool backgroundVisible READ backgroundVisible WRITE setBackgroundVisible)
  Q_PROPERTY(double backgroundOpacity READ backgroundOpacity WRITE setBackgroundOpacity)

public:
  explicit ArtificialHorizon(QWidget * parent = nullptr);
  ~ArtificialHorizon() override = default;

  // Getters
  double pitchAngle() const { return pitch_; }
  double rollAngle() const { return roll_; }
  bool backgroundVisible() const { return background_visible_; }
  double backgroundOpacity() const { return background_opacity_; }

  // Setters
  void setPitchAngle(double pitch);
  void setRollAngle(double roll);
  void setAttitude(double pitch, double roll);  // Convenience method
  void setBackgroundVisible(bool visible);
  void setBackgroundOpacity(double opacity);

  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  void drawSkyGround(QPainter & painter, double radius);
  void drawOuterRing(QPainter & painter, double radius);

  double pitch_;                // degrees, positive = nose up
  double roll_;                 // degrees, positive = right wing down
  bool background_visible_;     // show/hide background
  double background_opacity_;   // background opacity (0.0-1.0)
};

class PitchLadder : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(double pitchAngle READ pitchAngle WRITE setPitchAngle)
  Q_PROPERTY(double rollAngle READ rollAngle WRITE setRollAngle)
  Q_PROPERTY(double ladderRange READ ladderRange WRITE setLadderRange)
  Q_PROPERTY(double ladderStep READ ladderStep WRITE setLadderStep)
  Q_PROPERTY(bool visible READ isVisible WRITE setVisible)

public:
  explicit PitchLadder(QWidget * parent = nullptr);
  ~PitchLadder() override = default;

  // Getters
  double pitchAngle() const { return pitch_; }
  double rollAngle() const { return roll_; }
  double ladderRange() const { return ladder_range_; }
  double ladderStep() const { return ladder_step_; }

  // Setters
  void setPitchAngle(double pitch);
  void setRollAngle(double roll);
  void setLadderRange(double max_degrees);  // ±30, ±60, ±90
  void setLadderStep(double step);          // 5°, 10°, 15°, 20°

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  double pitch_;          // degrees, positive = nose up
  double roll_;           // degrees, positive = right wing down
  double ladder_range_;   // maximum pitch angle to display
  double ladder_step_;    // step between ladder lines
};

class AircraftReference : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(QColor color READ color WRITE setColor)
  Q_PROPERTY(bool visible READ isVisible WRITE setVisible)

public:
  explicit AircraftReference(QWidget * parent = nullptr);
  ~AircraftReference() override = default;

  // Getters
  QColor color() const { return color_; }

  // Setters
  void setColor(const QColor & color);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  QColor color_;
};

class RollIndicator : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(double rollAngle READ rollAngle WRITE setRollAngle)
  Q_PROPERTY(bool visible READ isVisible WRITE setVisible)

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
  double roll_;           // degrees, positive = right wing down
  double scale_factor_;   // scaling based on widget size
};

class AttitudeIndicator : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(bool showPitchLadder READ showPitchLadder WRITE setShowPitchLadder)
  Q_PROPERTY(bool showRollIndicator READ showRollIndicator WRITE setShowRollIndicator)
  Q_PROPERTY(bool showAircraftReference READ showAircraftReference WRITE setShowAircraftReference)
  Q_PROPERTY(double pitchLadderRange READ pitchLadderRange WRITE setPitchLadderRange)
  Q_PROPERTY(double pitchLadderStep READ pitchLadderStep WRITE setPitchLadderStep)
  Q_PROPERTY(bool backgroundVisible READ backgroundVisible WRITE setBackgroundVisible)
  Q_PROPERTY(double backgroundOpacity READ backgroundOpacity WRITE setBackgroundOpacity)

public:
  explicit AttitudeIndicator(QWidget * parent = nullptr);
  ~AttitudeIndicator() override = default;
  void setAttitude(double pitch, double roll);

  // Visibility getters
  bool showPitchLadder() const { return show_pitch_ladder_; }
  bool showRollIndicator() const { return show_roll_indicator_; }
  bool showAircraftReference() const { return show_aircraft_ref_; }
  
  // Configuration getters
  double pitchLadderRange() const { return pitch_ladder_range_; }
  double pitchLadderStep() const { return pitch_ladder_step_; }
  bool backgroundVisible() const { return background_visible_; }
  double backgroundOpacity() const { return background_opacity_; }

  // Visibility setters
  void setShowPitchLadder(bool show);
  void setShowRollIndicator(bool show);
  void setShowAircraftReference(bool show);
  
  // Configuration setters
  void setPitchLadderRange(double max_degrees);  // ±30, ±60, ±90
  void setPitchLadderStep(double step);          // 5°, 10°, 15°, 20°
  void setBackgroundVisible(bool visible);
  void setBackgroundOpacity(double opacity);

  QSize sizeHint() const override;

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
  void setupComponents();
  void updateComponentGeometry();

  // Component widgets
  ArtificialHorizon * horizon_;
  PitchLadder * pitch_ladder_;
  AircraftReference * aircraft_ref_;
  RollIndicator * roll_indicator_;

  // Configuration state
  bool show_pitch_ladder_;
  bool show_roll_indicator_;
  bool show_aircraft_ref_;
  double pitch_ladder_range_;
  double pitch_ladder_step_;
  bool background_visible_;
  double background_opacity_;
};

}  // namespace widgets
}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__WIDGETS__ATTITUDE_INDICATOR_HPP_
