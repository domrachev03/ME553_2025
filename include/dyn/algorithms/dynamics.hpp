#ifndef DYN_ALGORITHMS_DYNAMICS_HPP
#define DYN_ALGORITHMS_DYNAMICS_HPP

#include "../spatial.hpp"
#include "../structs.hpp"
#include "acceleration.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

namespace dyn {
namespace algorithms {

namespace dynamics {

inline void addGravityForce(const dyn::structs::Model &model,
                            dyn::structs::Data &data) {
  for (uint16_t i = 0; i < model.nl; ++i) {
    data.link_ext_force[i].setZero();
    data.link_ext_torque[i].setZero();
    data.link_ext_force[i] = model.link_mass[i] * data.gravity;
  }
}

inline Eigen::VectorXd
recursiveNewtonEulerAlgorithm(const dyn::structs::Model &model,
                              dyn::structs::Data &data,
                              const Eigen::VectorXd &dv) {
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  std::vector<Eigen::Vector<double, 6>> net_wrench(model.nj);
  Eigen::Matrix<double, 6, 6> I_c;

  uint16_t link_child_id, link_parent_id, jnt_parent_id, dof_addr;

  // First step -- compute fwd acceleration
  auto frames_acc =
      algorithms::acceleration::computeAcceleration(model, data, dv);
  std::vector<Eigen::Vector<double, 6>> jnt_acc = frames_acc.second;

  // Second step -- initialize net wrench with force acting on child link
  for (uint16_t jnt_id = 0; jnt_id < model.nj; ++jnt_id) {
    link_child_id = model.jnt_childid[jnt_id];
    Eigen::Vector3d r = data.link_i_pos[link_child_id] - data.jnt_pos[jnt_id];

    net_wrench[jnt_id].head(3) = data.link_ext_force[link_child_id];
    net_wrench[jnt_id].tail(3) =
        data.link_ext_torque[link_child_id] +
        spatial::skew_matrix(r) * data.link_ext_force[link_child_id];
  }

  for (int16_t jnt_id = model.nj - 1; jnt_id >= 0; --jnt_id) {
    link_child_id = model.jnt_childid[jnt_id];
    link_parent_id = model.jnt_parentid[jnt_id];
    jnt_parent_id = model.link_parentid[link_parent_id];
    dof_addr = model.jnt_dofadr[jnt_id];

    // First step -- compute tau for current joint
    // Subtree Spatial Inertia
    Eigen::Vector3d r = data.link_i_pos[link_child_id] - data.jnt_pos[jnt_id];
    I_c = spatial::construct_spatial_inertia(model.link_mass[link_child_id],
                                             data.link_I_w[link_child_id], r);
    Eigen::Vector<double, 6> bias;
    Eigen::Matrix3d omega_skew = spatial::skew_matrix(data.jnt_avel[jnt_id]);
    bias.head(3) = model.link_mass[link_child_id] * omega_skew * omega_skew * r;
    bias.tail(3) = omega_skew *
                   (data.link_I_w[link_child_id] -
                    model.link_mass[link_child_id] * spatial::skew_matrix(r) *
                        spatial::skew_matrix(r)) *
                   data.jnt_avel[jnt_id];
    auto f = I_c * jnt_acc[jnt_id] + bias + net_wrench[jnt_id];
    auto jnt_type = structs::JointType(model.jnt_type[jnt_id]);
    if (jnt_type != structs::FREE) {
      tau[dof_addr] = data.jnt_axis[jnt_id].dot(f);
    } else {
      tau(Eigen::seqN(dof_addr, 6)) = f;
    }
    // Then -- update parent's wrench
    if (jnt_parent_id != UINT16_MAX) {
      net_wrench[jnt_parent_id] += f;
      net_wrench[jnt_parent_id].tail(3) +=
          -spatial::skew_matrix(data.jnt_pos[jnt_parent_id] -
                                data.jnt_pos[jnt_id]) *
          f.head(3);
    }
  }

  return tau;
}

inline void computeBias(const dyn::structs::Model &model,
                        dyn::structs::Data &data) {
  Eigen::VectorXd dv = Eigen::VectorXd::Zero(model.nv);
  dv.head(3) = -data.gravity;
  data.b = recursiveNewtonEulerAlgorithm(model, data, dv);
}

} // namespace dynamics
} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_DYNAMICS_HPP