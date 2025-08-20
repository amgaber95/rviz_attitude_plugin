/*
 * Copyright (c) 2025
 * All rights reserved.
 *
 * RViz Attitude Display Plugin - Angle Readout Widget
 * 
 * Displays numeric angle values with a 3D glass effect.
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__ANGLE_READOUT_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__ANGLE_READOUT_HPP_

#include <QWidget>
#include <QColor>
#include <QString>

namespace rviz_attitude_plugin
{
namespace widgets
{

/**
 * @brief Numeric angle readout widget with glass effect styling
 * 
 * Displays a title and numeric value with sophisticated 3D glass effect:
 * - Gradient backgrounds for depth
 * - Shadow effects for raised appearance
 * - Configurable colors and titles
 * - Automatic value formatting with degree symbol
 */
class AngleReadout : public QWidget
{
  Q_OBJECT
  Q_PROPERTY(double value READ value WRITE setValue)
  Q_PROPERTY(QString title READ title WRITE setTitle)
  Q_PROPERTY(QColor color READ color WRITE setColor)
  Q_PROPERTY(QColor backgroundColor READ backgroundColor WRITE setBackgroundColor)

public:
  /**
   * @brief Construct a new Angle Readout widget
   * @param parent Parent widget
   */
  explicit AngleReadout(QWidget * parent = nullptr);

  /**
   * @brief Get current numeric value
   * @return Current value in degrees
   */
  double value() const { return value_; }

  /**
   * @brief Set numeric value to display
   * @param value Value in degrees
   */
  void setValue(double value);

  /**
   * @brief Get current title text
   * @return Title string
   */
  QString title() const { return title_; }

  /**
   * @brief Set title text
   * @param title Title to display above value
   */
  void setTitle(const QString & title);

  /**
   * @brief Get current text color
   * @return Text color
   */
  QColor color() const { return color_; }

  /**
   * @brief Set text color
   * @param color Color for title and value text
   */
  void setColor(const QColor & color);

  /**
   * @brief Get background color
   * @return Background base color
   */
  QColor backgroundColor() const { return background_color_; }

  /**
   * @brief Set background color
   * @param color Base color for glass effect background
   */
  void setBackgroundColor(const QColor & color);

  /**
   * @brief Get preferred size for the widget
   * @return Suggested size hint
   */
  QSize sizeHint() const override;

  /**
   * @brief Get minimum size for the widget
   * @return Minimum size hint
   */
  QSize minimumSizeHint() const override;

protected:
  /**
   * @brief Paint the readout with glass effect
   * @param event Paint event
   */
  void paintEvent(QPaintEvent * event) override;

private:
  /**
   * @brief Draw the glass effect background
   * @param painter QPainter reference
   * @param rect Rectangle to fill with glass effect
   */
  void drawGlassBackground(QPainter & painter, const QRectF & rect);

  /**
   * @brief Draw the title text
   * @param painter QPainter reference
   * @param rect Rectangle for title area
   */
  void drawTitle(QPainter & painter, const QRectF & rect);

  /**
   * @brief Draw the numeric value with formatting
   * @param painter QPainter reference
   * @param rect Rectangle for value area
   */
  void drawValue(QPainter & painter, const QRectF & rect);

  /**
   * @brief Draw shadow effects for depth
   * @param painter QPainter reference
   * @param rect Rectangle for shadow effects
   */
  void drawShadowEffects(QPainter & painter, const QRectF & rect);

  double value_;              ///< Current numeric value
  QString title_;             ///< Title text
  QColor color_;              ///< Text color
  QColor background_color_;   ///< Background base color
};

}  // namespace widgets
}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__WIDGETS__ANGLE_READOUT_HPP_
