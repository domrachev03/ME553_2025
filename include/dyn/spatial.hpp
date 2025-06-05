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

  rot = zRot * yRot * xRot;
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

// TODO: replace most skew matrices with cross product
inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
  Eigen::Matrix3d skew;
  skew << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return skew;
}

inline Eigen::Matrix3d move_I(const Eigen::Matrix3d &I,
                              const Eigen::Vector3d &r, const double &m) {
  return (I + m * (r.squaredNorm() * Eigen::Matrix3d::Identity() -
                   r * r.transpose()));
}

inline Eigen::Matrix<double, 6, 6>
get_dof_mapping_matrix(const Eigen::Vector3d &r) {
  Eigen::Matrix<double, 6, 6> X;
  X.setZero();
  X.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  X.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  X.block<3, 3>(3, 0) = -spatial::skew_matrix(r);

  return X;
}

inline Eigen::Matrix<double, 6, 6>
construct_spatial_inertia(const double &m, const Eigen::Matrix3d &I,
                          const Eigen::Vector3d &r) {
  Eigen::Matrix3d r_skew = spatial::skew_matrix(r);
  Eigen::Matrix3d m_r_skew = m * r_skew;
  Eigen::Matrix<double, 6, 6> J;
  J.setZero();
  J.block<3, 3>(0, 0) = m * Eigen::Matrix3d::Identity();
  J.block<3, 3>(3, 3) = I - m_r_skew * r_skew;
  J.block<3, 3>(0, 3) = -m_r_skew;
  J.block<3, 3>(3, 0) = m_r_skew;

  return J;
}
} // namespace spatial

} // namespace dyn
#endif // DYN_SPATIAL_HPP