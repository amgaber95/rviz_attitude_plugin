/*
 * RViz Attitude Display Plugin - Angle Readout Widget
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__WIDGETS__ANGLE_READOUT_HPP_
#define RVIZ_ATTITUDE_PLUGIN__WIDGETS__ANGLE_READOUT_HPP_

#include <QWidget>
#include <QString>

namespace rviz_attitude_plugin
{
namespace widgets
{

class AngleReadout : public QWidget
{
  Q_OBJECT

public:
  explicit AngleReadout(
    const QString & title,
    const QString & color = "#3B82F6",
    QWidget * parent = nullptr);
  ~AngleReadout() override = default;

  /**
   * @brief Update the displayed value.
   * @param text The text to display (typically a formatted number)
   */
  void setValue(const QString & text);

  /**
   * @brief Change the accent color.
   * @param color Color in hex format (e.g., "#3B82F6")
   */
  void setColor(const QString & color);

  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  void parseColor();

  QString color_;
  QString title_;
  QString value_;
  int r_, g_, b_;  // Parsed RGB values
};

}  // namespace widgets
}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__WIDGETS__ANGLE_READOUT_HPP_
