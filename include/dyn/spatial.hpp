#ifndef DYN_SPATIAL_HPP
#define DYN_SPATIAL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dyn {

namespace spatial {
inline Eigen::Matrix<double, 3, 3> rpy2rot(const Eigen::Vector3d &rpy) {
  Eigen::Matrix<double, 3, 3> rot;
  double thX = rpy[0], thY = rpy[1], thZ = rpy[2];

  Eigen::Matrix3d xRot, yRot, zRot;

  xRot << 1, 0, 0, 0, cos(thX), -sin(thX), 0, sin(thX), cos(thX);

  yRot << cos(thY), 0, sin(thY), 0, 1, 0, -sin(thY), 0, cos(thY);

  zRot << cos(thZ), -sin(thZ), 0, sin(thZ), cos(thZ), 0, 0, 0, 1;

  rot = xRot * yRot * zRot;
  return rot;
}

inline Eigen::Matrix3d axisangle2rot(const Eigen::Vector3d &axis) {
  Eigen::Matrix3d rot;
  double theta = axis.norm();
  Eigen::Vector3d n = axis.normalized();

  return Eigen::AngleAxisd(theta, n).toRotationMatrix();
}

inline Eigen::Matrix3d quat2rot(const Eigen::Vector4d &quat) {
  Eigen::Matrix3d rot;
  double w = quat[0], x = quat[1], y = quat[2], z = quat[3];

  rot << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
      2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
      2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);

  return rot;
}

inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
  Eigen::Matrix3d skew;
  skew << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return skew;
}
} // namespace spatial

} // namespace dyn
#endif // DYN_SPATIAL_HPP