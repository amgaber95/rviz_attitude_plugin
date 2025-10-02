/*
 * RViz Attitude Display Plugin - Euler Angle Converter (Header-Only)
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__EULER_CONVERTER_HPP_
#define RVIZ_ATTITUDE_PLUGIN__EULER_CONVERTER_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rviz_attitude_plugin
{

class EulerConverter
{
public:
  EulerConverter() = default;
  ~EulerConverter() = default;

  /**
   * @brief Convert quaternion to roll/pitch/yaw using ROS tf2::Matrix3x3(q).getRPY()
   * @param x, y, z, w Quaternion components
   * @param roll, pitch, yaw Output Euler angles in radians (ROS standard)
   */
  inline void convert(double x, double y, double z, double w,
                      double & roll, double & pitch, double & yaw) const
  {
    tf2::Quaternion q(x, y, z, w);
    if (q.length2() <= 0.0) {
      q.setValue(0.0, 0.0, 0.0, 1.0);
    } else {
      q.normalize();
    }
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }
};

}  // namespace rviz_attitude_plugin

#endif  // RVIZ_ATTITUDE_PLUGIN__EULER_CONVERTER_HPP_
