/*
 * RViz Attitude Display Plugin - Euler Angle Converter (Header-Only)
 *
 * Converts geometry_msgs quaternions into roll/pitch/yaw using tf2 utilities.
 */

#ifndef RVIZ_ATTITUDE_PLUGIN__EULER_CONVERTER_HPP_
#define RVIZ_ATTITUDE_PLUGIN__EULER_CONVERTER_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rviz_attitude_plugin
{

/**
 * @brief Header-only quaternion to Euler converter.
 *
 * The helper normalizes incoming quaternions (falling back to identity if
 * the quaternion has zero length) before using tf2's Matrix3x3::getRPY()
 * to extract roll, pitch and yaw in radians.
 */
class EulerConverter
{
public:
  EulerConverter() = default;
  ~EulerConverter() = default;

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

