/*
 * RViz Attitude Display Plugin - Heading Indicator Widget
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__HEADING_INDICATOR_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__HEADING_INDICATOR_HPP_

#include <QWidget>

namespace rviz_attitude_plugin
{
namespace widgets
{
class HeadingIndicator : public QWidget
{
  Q_OBJECT

public:
  explicit HeadingIndicator(QWidget * parent = nullptr);
  ~HeadingIndicator() override = default;

  void setHeading(double yaw);

  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  void draw3DCompassBezel(QPainter & painter, double radius);
  void drawFixedOuterRing(QPainter & painter, double radius);
  void drawRotatingCompassRose(QPainter & painter, double radius);

  double yaw_;            // degrees, ROS convention (0 = East, 90 = North)
  double scale_factor_;   // scaling based on widget size
};

}  // namespace widgets
}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__WIDGETS__HEADING_INDICATOR_HPP_
