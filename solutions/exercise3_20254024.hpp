#pragma once
#include <Eigen/Dense>

/// do not change the name of the method
inline Eigen::MatrixXd getMassMatrix(const Eigen::VectorXd &gc) {

  /// !!!!!!!!!! NO RAISIM FUNCTIONS HERE !!!!!!!!!!!!!!!!!

  return Eigen::MatrixXd::Ones(18, 18);
}