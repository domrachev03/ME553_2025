#ifndef DYN_ALGORITHMS_DYNAMICS_HPP
#define DYN_ALGORITHMS_DYNAMICS_HPP

#include "../spatial.hpp"
#include "../structs.hpp"
#include "Eigen/src/Core/Matrix.h"
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

    data.link_spatial_M[link_id] = spatial::construct_spatial_inertia(
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
    f = data.link_spatial_M[link_child_id] * jnt_acc[jnt_id] +
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
  data.articulated_M = data.link_spatial_M;
  data.articulated_b = data.link_spatial_b;

  uint16_t j_id, l_p_id, j_dof;
  Eigen::Vector<double, 6> aa;
  Eigen::Vector<double, 3> r, v;

  std::vector<Eigen::Vector<double, 6>> aa_dot(model.nl);
  std::vector<Eigen::Matrix<double, 6, 6>> X_bp(model.nl), X_bp_dot(model.nl);
  std::vector<double> SMS_inv(model.nl);

  for (int16_t l_id = model.nl - 1; l_id >= 0; --l_id) {
    j_id = model.link_parentid[l_id];
    if (j_id == UINT16_MAX) {
      // Base link, no parent joint
      continue;
    }
    l_p_id = model.jnt_parentid[j_id];
    j_dof = model.jnt_dofadr[j_id];

    Eigen::Vector<double, 6> link_v = Eigen::Vector<double, 6>::Zero();
    link_v.head(3) = data.link_lvel[l_p_id];
    link_v.tail(3) = data.link_avel[l_p_id];
    Eigen::Vector<double, 6> jnt_v = Eigen::Vector<double, 6>::Zero();
    jnt_v.head(3) = data.jnt_lvel[j_id];
    jnt_v.tail(3) = data.jnt_avel[j_id];

    aa = data.jnt_axis[j_id];
    aa_dot[l_id] = aa;
    aa_dot[l_id].head(3) =
        spatial::skew_matrix(data.jnt_avel[j_id]) * aa_dot[l_id].head(3);
    aa_dot[l_id].tail(3) =
        spatial::skew_matrix(data.jnt_avel[j_id]) * aa_dot[l_id].tail(3);
    SMS_inv[l_id] =
        (aa.transpose() * data.articulated_M[l_id] * aa).inverse()(0, 0);

    if (model.link_parentid[l_p_id] != UINT16_MAX) {
      r = -data.jnt_pos[j_id] + data.jnt_pos[model.link_parentid[l_p_id]];
    } else {
      r = -data.jnt_pos[j_id];
    }
    // FIXME: maybe not true for floating joint
    v = aa.head<3>() * data.v[j_dof];
    X_bp[l_id] = spatial::get_dof_mapping_matrix(r);

    X_bp_dot[l_id] = Eigen::Matrix<double, 6, 6>::Zero();
    X_bp_dot[l_id].block<3, 3>(3, 0) =
        spatial::skew_matrix(-data.link_avel[l_p_id].cross(r) + v);

    if (model.jnt_type[j_id] == structs::FIXED) {
      data.articulated_M[l_p_id] +=
          X_bp[l_id] * data.articulated_M[l_id] * X_bp[l_id].transpose();
      data.articulated_b[l_p_id] +=
          X_bp[l_id] *
          (data.articulated_M[l_id] * X_bp_dot[l_id].transpose() * link_v +
           data.articulated_b[l_id]);
    } else if (model.jnt_type[j_id] != structs::FREE) {
      data.articulated_M[l_p_id] +=
          X_bp[l_id] * data.articulated_M[l_id] *
          (-aa * SMS_inv[l_id] *
               (aa.transpose() * data.articulated_M[l_id] *
                X_bp[l_id].transpose()) +
           X_bp[l_id].transpose());
      data.articulated_b[l_p_id] +=
          X_bp[l_id] * (data.articulated_M[l_id] *
                            (aa * SMS_inv[l_id] *
                                 (data.tau[j_dof] -
                                  aa.transpose() * data.articulated_M[l_id] *
                                      (aa_dot[l_id] * data.v[j_dof] +
                                       X_bp_dot[l_id].transpose() * link_v) -
                                  aa.transpose() * data.articulated_b[l_id]) +
                             aa_dot[l_id] * data.v[j_dof] +
                             X_bp_dot[l_id].transpose() * link_v) +
                        data.articulated_b[l_id]);
    }
  }

  Eigen::VectorXd dv = Eigen::VectorXd::Zero(model.nv);
  std::vector<Eigen::Vector<double, 6>> link_acc(
      model.nl, Eigen::Vector<double, 6>::Zero());
  link_acc[0].head(3) = -data.gravity;

  for (int16_t l_id = 0; l_id < model.nl; ++l_id) {
    j_id = model.link_parentid[l_id];
    if (j_id == UINT16_MAX) {
      // Base link, no parent joint
      continue;
    }
    l_p_id = model.jnt_parentid[j_id];
    j_dof = model.jnt_dofadr[j_id];

    Eigen::Vector<double, 6> link_v = Eigen::Vector<double, 6>::Zero();
    link_v.head(3) = data.link_lvel[l_p_id];
    link_v.tail(3) = data.link_avel[l_p_id];
    Eigen::Vector<double, 6> jnt_v = Eigen::Vector<double, 6>::Zero();
    jnt_v.head(3) = data.jnt_lvel[j_id];
    jnt_v.tail(3) = data.jnt_avel[j_id];

    aa = data.jnt_axis[j_id];

    if (model.jnt_type[j_id] == structs::FIXED) {
      link_acc[l_id] = link_acc[l_p_id];
    } else if (model.jnt_type[j_id] == structs::FREE) {
      dv.segment<6>(j_dof) =
          data.articulated_M[l_id].inverse() *
          (data.tau.segment<6>(j_dof) - data.articulated_b[l_id]);
      dv.segment<3>(j_dof) += data.gravity;

      link_acc[l_id] = dv.segment<6>(j_dof) +
                       X_bp_dot[l_id].transpose() * link_v +
                       X_bp[l_id].transpose() * link_acc[l_p_id];
    } else {
      dv[j_dof] =
          SMS_inv[l_id] * (data.tau[j_dof] -
                           aa.transpose() * data.articulated_M[l_id] *
                               (aa_dot[l_id] * data.v[j_dof] +
                                X_bp_dot[l_id].transpose() * link_v +
                                X_bp[l_id].transpose() * link_acc[l_p_id]) -
                           aa.transpose() * data.articulated_b[l_id]);
      link_acc[l_id] = aa * dv[j_dof] + aa_dot[l_id] * data.v[j_dof] +
                       X_bp_dot[l_id].transpose() * link_v +
                       X_bp[l_id].transpose() * link_acc[l_p_id];
    }
  }

  data.dv = dv;
}

inline void
articulatedBodyalgorithmFeatherstone(const dyn::structs::Model &model,
                                     dyn::structs::Data &data) {
  // NOTE: In contrast with Featherstone, everything is computed in world frame
  //       similarly to the most other dyn's code.
  //       Body inertia and bias are already computed above, so their
  //       computation is skipped.
  // Step 1: compute velocity-product accelerations and SRB bias forces
  std::vector<Eigen::Vector<double, 6>> c(model.nl,
                                          Eigen::Vector<double, 6>::Zero());
  Eigen::Vector<double, 6> aa, aa_dot, link_vel;
  Eigen::Matrix<double, 6, 6> aa_floating = Eigen::Matrix<double, 6, 6>::Zero();

  uint16_t j_id, l_p_id, j_dof;
  for (uint16_t l_id = 0; l_id < model.nl; ++l_id) {
    j_id = model.link_parentid[l_id];
    if (j_id == UINT16_MAX) {
      // Base link, no parent joint
      continue;
    }
    j_dof = model.jnt_dofadr[j_id];
    l_p_id = model.jnt_parentid[j_id];

    link_vel.head(3) = data.link_lvel[l_id];
    link_vel.tail(3) = data.link_avel[l_id];
    aa = data.jnt_axis[j_id];
    // FIXME: wrong velocity
    aa_dot = spatial::cross6(link_vel, aa);

    if (model.jnt_type[j_id] == structs::FREE) {
      aa_floating.block<3, 3>(0, 0) = data.jnt_rot[j_id];
      aa_floating.block<3, 3>(3, 3) = data.jnt_rot[j_id];
      // TODO: is this vector still zero in world frame?
      // c[l_id] =
      //     aa_dot.cwiseProduct(data.v.segment<6>(j_dof)) +
      //     spatial::cross6(link_vel, aa_floating * data.v.segment<6>(j_dof));
    } else if (model.jnt_type[j_id] == structs::FIXED) {
    } else {
      // TODO: is this vector still zero in world frame?
      // c[l_id] = aa_dot * data.v[j_dof] +
      //           spatial::cross6(link_vel, aa * data.v[j_dof]);
      // c[l_id] = aa_dot * data.v[j_dof];
      std::cout << "Link " << l_id << " c: " << c[l_id].transpose() << "\n";
    }
  }

  // Step 2: calculate articulated body inertia and bias
  std::vector<Eigen::Matrix<double, 6, 6>> I_A = data.link_spatial_M;
  std::vector<Eigen::Vector<double, 6>> b_A = data.link_spatial_b;

  Eigen::Matrix<double, 6, 6> I_a;

  for (int16_t l_id = model.nl - 1; l_id >= 0; --l_id) {
    j_id = model.link_parentid[l_id];
    if (j_id == UINT16_MAX) {
      // Base link, no parent joint
      continue;
    }
    l_p_id = model.jnt_parentid[j_id];
    j_dof = model.jnt_dofadr[j_id];

    aa = data.jnt_axis[j_id];
    if (model.jnt_type[j_id] == structs::FIXED) {
      I_A[l_p_id] += I_A[l_id];
      b_A[l_p_id] += b_A[l_id] + I_A[l_id] * c[l_id];
    } else if (model.jnt_type[j_id] == structs::FREE) {
      // FIXME: bruh...
      aa_floating.block<3, 3>(0, 0) = data.jnt_rot[j_id];
      aa_floating.block<3, 3>(3, 3) = data.jnt_rot[j_id];

      Eigen::Matrix<double, 6, 6> SIS_inv =
          (aa_floating.transpose() * I_A[l_id] * aa_floating).inverse();
      Eigen::Matrix<double, 6, 6> IS_SIS_inv =
          I_A[l_id] * aa_floating * SIS_inv;

      I_a = I_A[l_id] - IS_SIS_inv * aa_floating.transpose() * I_A[l_id];
      I_A[l_p_id] += I_a;
      b_A[l_p_id] += b_A[l_id] + I_a * c[l_id] +
                     IS_SIS_inv * (data.tau.segment<6>(j_dof) -
                                   aa_floating.transpose() * b_A[l_id]);
    } else {
      auto SIS_inv = (aa.transpose() * I_A[l_id] * aa).inverse();
      auto IS_SIS_inv = I_A[l_id] * aa * SIS_inv;

      I_a = I_A[l_id] - IS_SIS_inv * aa.transpose() * I_A[l_id];
      I_A[l_p_id] += I_a;
      b_A[l_p_id] +=
          b_A[l_id] + I_a * c[l_id] +
          IS_SIS_inv * (data.tau[j_dof] - aa.transpose() * b_A[l_id]);
    }
  }

  // Step 3: compute joint torques
  Eigen::VectorXd dv = Eigen::VectorXd::Zero(model.nv);
  std::vector<Eigen::Vector<double, 6>> link_acc(
      model.nl, Eigen::Vector<double, 6>::Zero());
  link_acc[0].head(3) = -data.gravity;

  for (uint16_t l_id = 0; l_id < model.nl; ++l_id) {
    uint16_t j_id = model.link_parentid[l_id];
    if (j_id == UINT16_MAX) {
      // Base link, no parent joint
      continue;
    }
    uint16_t l_p_id = model.jnt_parentid[j_id];
    uint16_t j_dof = model.jnt_dofadr[j_id];

    if (model.jnt_type[j_id] == structs::FIXED) {
      link_acc[l_id] = link_acc[l_p_id];
    } else if (model.jnt_type[j_id] == structs::FREE) {
      // FIXME: bruh...
      aa_floating.block<3, 3>(0, 0) = data.jnt_rot[j_id];
      aa_floating.block<3, 3>(3, 3) = data.jnt_rot[j_id];

      Eigen::Matrix<double, 6, 6> SIS_inv =
          (aa_floating.transpose() * I_A[l_id] * aa_floating).inverse();
      Eigen::Matrix<double, 6, 6> IS_SIS_inv =
          I_A[l_id] * aa_floating * SIS_inv;

      dv.segment<6>(j_dof) = SIS_inv * (data.tau.segment<6>(j_dof) -
                                        aa_floating.transpose() * I_A[l_id] *
                                            (link_acc[l_p_id] + c[l_id]) -
                                        aa_floating.transpose() * b_A[l_id]);
      link_acc[l_id] =
          link_acc[l_p_id] + c[l_id] + aa_floating * dv.segment<6>(j_dof);
    } else {
      aa = data.jnt_axis[j_id];
      auto SIS_inv = (aa.transpose() * I_A[l_id] * aa).inverse();

      dv[j_dof] = (SIS_inv *
                   (data.tau[j_dof] -
                    aa.transpose() * I_A[l_id] * (link_acc[l_p_id] + c[l_id]) -
                    aa.transpose() * b_A[l_id]))(0, 0);
      link_acc[l_id] = link_acc[l_p_id] + c[l_id] + aa * dv[j_dof];
    }
  }

  data.dv = dv;
}

} // namespace dynamics
} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_DYNAMICS_HPP