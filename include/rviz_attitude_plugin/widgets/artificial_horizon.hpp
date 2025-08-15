/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Artificial Horizon Widget
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__ARTIFICIAL_HORIZON_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__ARTIFICIAL_HORIZON_HPP_

#include <QWidget>

namespace rviz_attitude_plugin
{
namespace widgets
{

/**
 * @brief Artificial horizon showing sky/ground gradient.
 * 
 * Displays a rotating horizon line with blue sky above and brown ground below.
 * Rotates and translates based on pitch and roll angles.
 */
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
  void setAttitude(double pitch, double roll);
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
  bool background_visible_;
  double background_opacity_;
};

}  // namespace widgets
}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__WIDGETS__ARTIFICIAL_HORIZON_HPP_
