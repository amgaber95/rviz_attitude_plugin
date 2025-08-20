/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Attitude Indicator Components
 * 
 * This file consolidates all attitude indicator components into a single header
 * while maintaining separate, focused classes for modularity:
 * - ArtificialHorizon: Sky/ground gradient background
 * - AircraftReference: Fixed center crosshair symbol
 * - RollIndicator: Roll scale arc with pointer
 * - AttitudeIndicator: Composite widget coordinating all components
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__ATTITUDE_INDICATOR_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__ATTITUDE_INDICATOR_HPP_

#include <QWidget>
#include <QColor>

namespace rviz_attitude_plugin
{
namespace widgets
{

// ============================================================================
// ArtificialHorizon - Sky/ground gradient background
// ============================================================================

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

// ============================================================================
// AircraftReference - Fixed center crosshair symbol
// ============================================================================

/**
 * @brief Aircraft reference symbol (center crosshair/pointer).
 * 
 * Displays the aircraft symbol that represents the aircraft's position
 * relative to the horizon. Remains fixed in the center while the horizon
 * rotates around it.
 */
class AircraftReference : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(QColor color READ color WRITE setColor)

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

// ============================================================================
// RollIndicator - Roll scale arc with pointer
// ============================================================================

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

// ============================================================================
// AttitudeIndicator - Composite widget coordinating all components
// ============================================================================

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
