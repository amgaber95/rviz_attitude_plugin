/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Heading Indicator Widget
 * 
 * Displays a rotating compass rose with cardinal directions and degree markers.
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__HEADING_INDICATOR_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__HEADING_INDICATOR_HPP_

#include <QWidget>

namespace rviz_attitude_plugin
{
namespace widgets
{

/**
 * @brief Compass rose widget displaying heading/yaw angle
 * 
 * Displays a rotating compass rose with:
 * - Cardinal directions (N, E, S, W) and intermediate directions (NE, SE, SW, NW)
 * - Degree markers at major intervals (0°, 30°, 60°, etc.)
 * - Tick marks for minor intervals
 * - Fixed center pointer indicating current heading
 * - Optional numeric heading text display
 */
class HeadingIndicator : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(double headingAngle READ headingAngle WRITE setHeadingAngle)
  Q_PROPERTY(bool showText READ showText WRITE setShowText)

public:
  /**
   * @brief Construct a new Heading Indicator widget
   * @param parent Parent widget
   */
  explicit HeadingIndicator(QWidget * parent = nullptr);

  /**
   * @brief Get current heading angle in degrees
   * @return Heading angle (0° = North, 90° = East, 180° = South, 270° = West)
   */
  double headingAngle() const { return heading_; }

  /**
   * @brief Set heading angle
   * @param heading Heading angle in degrees (0-360, or any value - will be normalized)
   */
  void setHeadingAngle(double heading);

  /**
   * @brief Get whether numeric heading text is shown
   * @return true if text is displayed
   */
  bool showText() const { return show_text_; }

  /**
   * @brief Set whether to display numeric heading text
   * @param show true to show text, false to hide
   */
  void setShowText(bool show);

  /**
   * @brief Get preferred size for the widget
   * @return Suggested size hint
   */
  QSize sizeHint() const override;

protected:
  /**
   * @brief Paint the compass rose
   * @param event Paint event
   */
  void paintEvent(QPaintEvent * event) override;

private:
  /**
   * @brief Draw the rotating compass rose ring
   * @param painter QPainter reference
   * @param radius Compass rose radius
   */
  void drawCompassRose(QPainter & painter, double radius);

  /**
   * @brief Draw cardinal and intermediate direction labels
   * @param painter QPainter reference
   * @param radius Compass rose radius
   */
  void drawDirectionLabels(QPainter & painter, double radius);

  /**
   * @brief Draw degree markers around the compass
   * @param painter QPainter reference
   * @param radius Compass rose radius
   */
  void drawDegreeMarkers(QPainter & painter, double radius);

  /**
   * @brief Draw the fixed center heading pointer
   * @param painter QPainter reference
   * @param radius Compass rose radius
   */
  void drawHeadingPointer(QPainter & painter, double radius);

  /**
   * @brief Draw numeric heading text at the bottom
   * @param painter QPainter reference
   */
  void drawHeadingText(QPainter & painter);

  /**
   * @brief Normalize angle to 0-360 range
   * @param angle Input angle in degrees
   * @return Normalized angle [0, 360)
   */
  double normalizeAngle(double angle) const;

  double heading_;      ///< Current heading angle in degrees
  bool show_text_;      ///< Whether to display numeric heading text
};

}  // namespace widgets
}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__WIDGETS__HEADING_INDICATOR_HPP_
