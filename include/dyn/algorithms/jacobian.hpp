#ifndef DYN_ALGORITHMS_JACOBIAN_HPP
#define DYN_ALGORITHMS_JACOBIAN_HPP

#include "../spatial.hpp"
#include "../structs.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

namespace dyn {
namespace algorithms {

namespace kinematics {
inline Eigen::MatrixXd
computeLinearJacobian(const dyn::structs::Model &model,
                      const dyn::structs::Data &data, const uint16_t &obj_id,
                      //  TODO: this should be implemented differently...
                      const bool &is_jnt, const Eigen::Vector3d &point) {

  Eigen::MatrixXd Jlin(3, model.nv);
  Jlin.setZero();
  Eigen::Vector3d r_ee;
  uint16_t jnt_id;
  if (is_jnt) {
    jnt_id = obj_id;
    r_ee = data.jnt_pos[jnt_id] + point;
  } else {
    jnt_id = model.link_parentid[obj_id];
    r_ee = data.link_i_pos[obj_id] + point;
  }
  while (jnt_id != UINT16_MAX) {
    Eigen::Vector3d r_jnt_ee = r_ee - data.jnt_pos[jnt_id];
    uint16_t dof_idx = model.jnt_dofadr[jnt_id];

    // TODO: does not support floating base
    auto jnt_type = model.jnt_type[jnt_id];
    if (jnt_type == structs::REVOLUTE || jnt_type == structs::PRISMATIC)
      Jlin.col(dof_idx) +=
          data.jnt_axis_pos[jnt_id] + data.jnt_axis_rot[jnt_id].cross(r_jnt_ee);
    else if (jnt_type == structs::FREE) {
      Jlin(Eigen::all, Eigen::seqN(dof_idx, 3)) += Eigen::Matrix3d::Identity();
      Jlin(Eigen::all, Eigen::seqN(dof_idx + 3, 3)) +=
          -spatial::skew_matrix(r_jnt_ee);
    }
    jnt_id = model.link_parentid[model.jnt_parentid[jnt_id]];
  }

  return Jlin;
}
inline Eigen::MatrixXd computeAngularJacobian(const dyn::structs::Model &model,
                                              const dyn::structs::Data &data,
                                              const uint16_t &obj_id,
                                              const bool &is_jnt) {
  Eigen::MatrixXd Jang(3, model.nv);
  Jang.setZero();
  uint16_t jnt_id;
  if (is_jnt) {
    jnt_id = obj_id;
  } else {
    jnt_id = model.link_parentid[obj_id];
  }
  while (jnt_id != UINT16_MAX) {
    uint16_t dof_idx = model.jnt_dofadr[jnt_id];

    // TODO: does not support floating base
    auto jnt_type = model.jnt_type[jnt_id];
    if (jnt_type == structs::REVOLUTE || jnt_type == structs::PRISMATIC)
      Jang.col(dof_idx) += data.jnt_axis_rot[jnt_id];
    else if (jnt_type == structs::FREE) {
      Jang(Eigen::all, Eigen::seqN(dof_idx + 3, 3)) +=
          Eigen::Matrix3d::Identity();
    }
    jnt_id = model.link_parentid[model.jnt_parentid[jnt_id]];
  }

  return Jang;
}

inline Eigen::MatrixXd computeLinearJacobian(const dyn::structs::Model &model,
                                             const dyn::structs::Data &data,
                                             const uint16_t &jnt_id) {
  return computeLinearJacobian(model, data, jnt_id, true,
                               Eigen::Vector3d::Zero());
}

inline Eigen::MatrixXd computeAngularJacobian(const dyn::structs::Model &model,
                                              const dyn::structs::Data &data,
                                              const uint16_t &jnt_id) {
  return computeAngularJacobian(model, data, jnt_id, true);
}

inline Eigen::MatrixXd computeJacobian(const dyn::structs::Model &model,
                                       dyn::structs::Data &data,
                                       const uint16_t &obj_id,
                                       const bool &is_jnt,
                                       const Eigen::Vector3d &point) {
  Eigen::MatrixXd J(6, model.nv);
  J.block(0, 0, 3, model.nv) =
      computeLinearJacobian(model, data, obj_id, is_jnt, point);
  J.block(3, 0, 3, model.nv) =
      computeAngularJacobian(model, data, obj_id, is_jnt);
  return J;
}

inline void computeJandVel(const dyn::structs::Model &model,
                           dyn::structs::Data &data) {
  for (uint16_t i = 0; i < model.nl; ++i) {
    data.link_Jpos[i] =
        computeLinearJacobian(model, data, i, false, Eigen::Vector3d::Zero());
    data.link_lvel[i] = data.link_Jpos[i] * data.v;

    data.link_Jrot[i] = computeAngularJacobian(model, data, i, false);
    data.link_avel[i] = data.link_Jrot[i] * data.v;
  }

  for (uint16_t i = 0; i < model.nj; ++i) {
    data.jnt_Jpos[i] = computeLinearJacobian(model, data, i);
    data.jnt_lvel[i] = data.jnt_Jpos[i] * data.v;

    data.jnt_Jrot[i] = computeAngularJacobian(model, data, i);
    data.jnt_avel[i] = data.jnt_Jrot[i] * data.v;
  }
}

} // namespace kinematics
} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_JACOBIAN_HPP