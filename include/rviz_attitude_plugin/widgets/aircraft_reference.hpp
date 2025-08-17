/*
 * RViz Attitude Display Plugin - Aircraft Reference Widget
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__AIRCRAFT_REFERENCE_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__AIRCRAFT_REFERENCE_HPP_

#include <QWidget>
#include <QColor>

namespace rviz_attitude_plugin
{
namespace widgets
{

/**
 * @brief Aircraft reference symbol (center crosshair/pointer).
 * 
 * Displays the aircraft symbol that represents the aircraft's position
 * relative to the horizon. Remains fixed in the center while the horizon
 * and pitch ladder rotate around it.
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

}  // namespace widgets
}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__WIDGETS__AIRCRAFT_REFERENCE_HPP_
