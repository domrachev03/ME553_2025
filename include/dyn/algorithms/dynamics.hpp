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

inline void computeSingleBodyNewtonEuler(const dyn::structs::Model &model,
                                         dyn::structs::Data &data) {
  uint16_t link_id;
  Eigen::Vector3d r;
  Eigen::Matrix3d omega_skew, omega_r;
  for (uint16_t jnt_id = 0; jnt_id < model.nj; ++jnt_id) {
    link_id = model.jnt_childid[jnt_id];
    r = data.link_i_pos[link_id] - data.jnt_pos[jnt_id];

    data.link_spatial_I[link_id] = spatial::construct_spatial_inertia(
        model.link_mass[link_id], data.link_I_w[link_id], r);
    omega_skew = spatial::skew_matrix(data.jnt_avel[jnt_id]);
    omega_r = spatial::skew_matrix(r);
    data.link_spatial_b[link_id].head(3) =
        model.link_mass[link_id] * omega_skew * omega_skew * r;
    data.link_spatial_b[link_id].tail(3) =
        omega_skew *
        (data.link_I_w[link_id] -
         model.link_mass[link_id] * omega_r * omega_r) *
        data.jnt_avel[jnt_id];
  }
}

inline Eigen::VectorXd recursiveNewtonEulerAlgorithm(
    const dyn::structs::Model &model, dyn::structs::Data &data,
    const Eigen::VectorXd &dv, const bool &include_external) {
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
  std::vector<Eigen::Vector<double, 6>> net_wrench(
      model.nj, Eigen::Vector<double, 6>::Zero());

  uint16_t link_child_id, link_parent_id, jnt_parent_id, dof_addr;

  // First step -- compute fwd acceleration
  auto frames_acc =
      algorithms::acceleration::computeAcceleration(model, data, dv);
  std::vector<Eigen::Vector<double, 6>> jnt_acc = frames_acc.second;

  // Second step -- initialize net wrench with force acting on child link
  if (include_external) {
    for (uint16_t jnt_id = 0; jnt_id < model.nj; ++jnt_id) {
      link_child_id = model.jnt_childid[jnt_id];

      net_wrench[jnt_id] = data.link_ext_wrench[link_child_id];
      net_wrench[jnt_id].tail(3) +=
          spatial::skew_matrix(data.link_i_pos[link_child_id] -
                               data.jnt_pos[jnt_id]) *
          data.link_ext_wrench[link_child_id].head(3);
    }
  }

  Eigen::Vector3d r;
  Eigen::Matrix3d omega_skew, omega_r;
  Eigen::Vector<double, 6> f;
  structs::JointType jnt_type;
  for (int16_t jnt_id = model.nj - 1; jnt_id >= 0; --jnt_id) {
    link_child_id = model.jnt_childid[jnt_id];
    link_parent_id = model.jnt_parentid[jnt_id];
    jnt_parent_id = model.link_parentid[link_parent_id];

    // First step -- compute tau for current joint
    // Subtree Spatial Inertia
    r = data.link_i_pos[link_child_id] - data.jnt_pos[jnt_id];
    omega_skew = spatial::skew_matrix(data.jnt_avel[jnt_id]);
    omega_r = spatial::skew_matrix(r);
    f = data.link_spatial_I[link_child_id] * jnt_acc[jnt_id] +
        data.link_spatial_b[link_child_id] + net_wrench[jnt_id];
    jnt_type = structs::JointType(model.jnt_type[jnt_id]);

    dof_addr = model.jnt_dofadr[jnt_id];
    if (jnt_type == structs::FREE) {
      tau(Eigen::seqN(dof_addr, 6)) = f;
    } else if (jnt_type != structs::FIXED) {
      tau[dof_addr] = data.jnt_axis[jnt_id].dot(f);
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
  data.b = recursiveNewtonEulerAlgorithm(model, data, dv, false);
}

inline void articulatedBodyAlgorithmHwangbo(const dyn::structs::Model &model,
                                            dyn::structs::Data &data) {
  // Step 1: compute articulated body inertia and bias
  for (uint16_t l_id = 0; l_id < model.nl; ++l_id) {
    data.articulated_M[l_id] = data.link_spatial_I[l_id];
    data.articulated_b[l_id] = data.link_spatial_b[l_id];
  }
  Eigen::Matrix<double, 6, 6> X_bp,
      X_bp_dot = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Vector<double, 6> aa, aa_dot, link_vel, aaM_inv;
  uint16_t j_id, j_dof, l_p_id;
  for (int16_t l_id = model.nl - 1; l_id >= 1; --l_id) {
    std::cout << "Link Inertia: " << l_id << "\n"
              << data.articulated_M[l_id] << "\n";
    std::cout << "Link Bias: " << l_id << std::endl
              << data.articulated_b[l_id].transpose() << "\n";
    j_id = model.link_parentid[l_id];
    if (j_id == UINT16_MAX) {
      // Base link, no parent joint
      continue;
    }
    j_dof = model.jnt_dofadr[j_id];
    l_p_id = model.jnt_parentid[j_id];
    X_bp = -1 * spatial::get_dof_mapping_matrix(data.link_i_pos[l_id] -
                                                data.link_i_pos[l_p_id]);
    X_bp_dot.block<3, 3>(3, 0) =
        spatial::skew_matrix(data.link_lvel[l_id] - data.link_lvel[l_p_id]);

    aa = data.jnt_axis[j_id];
    aa_dot.head(3) = spatial::skew_matrix(data.jnt_avel[j_id]) * aa.head(3);
    aa_dot.tail(3) = spatial::skew_matrix(data.jnt_avel[j_id]) * aa.tail(3);
    aaM_inv = aa * (aa.transpose() * data.articulated_M[l_id] * aa).inverse();

    link_vel.head(3) = data.link_lvel[l_p_id];
    link_vel.tail(3) = data.link_avel[l_p_id];

    data.articulated_M[l_p_id] +=
        X_bp * data.articulated_M[l_id] *
        (-aaM_inv *
             (aa.transpose() * data.articulated_M[l_id] * X_bp.transpose()) +
         X_bp.transpose());

    data.articulated_b[l_p_id] +=
        X_bp * (data.articulated_M[l_id] *
                    (aaM_inv * (data.tau[j_dof] -
                                aa.transpose() * data.articulated_M[l_id] *
                                    (aa_dot * data.v[j_dof] +
                                     X_bp_dot.transpose() * link_vel) -
                                aa.transpose() * data.articulated_b[l_id]) +
                     aa_dot * data.v[j_dof] + X_bp_dot.transpose() * link_vel) +
                data.articulated_b[l_id]);
  }

  // Step 2: compute joint torques
  Eigen::Matrix<double, 1, 1> u_dot;
  Eigen::Vector<double, 6> w_dot;
  bool is_first = true;

  for (uint16_t l_id = 1; l_id < model.nl; ++l_id) {
    uint16_t j_id = model.link_parentid[l_id];
    uint16_t l_p_id = model.jnt_parentid[j_id];
    uint16_t j_dof = model.jnt_dofadr[j_id];

    if (model.jnt_type[j_id] == structs::FIXED)
      continue;

    if (is_first) {
      is_first = false;
      // FIXME: only for floating base
      w_dot = data.articulated_M[l_id].inverse() *
              (data.tau.segment<6>(j_dof) - data.articulated_b[l_id]);
      data.dv.segment<6>(j_dof) = w_dot;
      continue;
    }

    X_bp = spatial::get_dof_mapping_matrix(data.link_i_pos[l_id] -
                                           data.link_i_pos[l_p_id]);
    aa = data.jnt_axis[j_id];
    aa_dot.head(3) = spatial::skew_matrix(data.jnt_avel[j_id]) * aa.head(3);
    aa_dot.tail(3) = spatial::skew_matrix(data.jnt_avel[j_id]) * aa.tail(3);
    X_bp_dot.block<3, 3>(3, 0) =
        -spatial::skew_matrix(data.link_lvel[l_id] - data.link_lvel[l_p_id]);

    link_vel.head(3) = data.link_lvel[l_p_id];
    link_vel.tail(3) = data.link_avel[l_p_id];

    u_dot = (aa.transpose() * data.articulated_M[l_id] * aa).inverse() *
            (data.tau[j_dof] -
             aa.transpose() * data.articulated_M[l_id] *
                 (aa_dot * data.v[j_dof] + X_bp_dot.transpose() * link_vel +
                  X_bp.transpose() * w_dot) -
             aa.transpose() * data.articulated_b[l_id]);
    w_dot = aa * data.v[j_dof] + X_bp.transpose() * w_dot;

    // FIXME: something is wrong
    data.dv[j_dof] = u_dot(0, 0);
  }
}

} // namespace dynamics
} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_DYNAMICS_HPP