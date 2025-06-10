#ifndef DYN_ALGORITHMS_KINEMATICS_HPP
#define DYN_ALGORITHMS_KINEMATICS_HPP

#include "../spatial.hpp"
#include "../structs.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <iostream>
#include <vector>

namespace dyn {
namespace algorithms {

namespace kinematics {

inline void computeForwardKinematics(const dyn::structs::Model &model,
                                     dyn::structs::Data &data) {
  data.jnt_pos[0] = Eigen::Vector3d::Zero();
  data.jnt_rot[0] = Eigen::Matrix3d::Identity();
  for (uint16_t jnt_id = 0; jnt_id < model.nj; ++jnt_id) {
    // Get the parent frame
    uint16_t link_parent_id = model.jnt_parentid[jnt_id];
    uint16_t jnt_parent_id = model.link_parentid[link_parent_id];
    if (jnt_parent_id == UINT16_MAX) {
      jnt_parent_id = 0; // Set to base if no parent
    }
    Eigen::Vector3d parent_pos = data.jnt_pos[jnt_parent_id];
    Eigen::Matrix3d parent_rot = data.jnt_rot[jnt_parent_id];

    uint16_t child_id = model.jnt_childid[jnt_id];
    // Compute the forward kinematics for each joint
    const auto &jnt_pos = model.jnt_rel_pos[jnt_id];
    const auto &joint_rot = model.jnt_rel_rot[jnt_id];

    data.jnt_pos[jnt_id] = parent_pos + parent_rot * jnt_pos;
    data.jnt_rot[jnt_id] = parent_rot * joint_rot;

    // Handle joint effect
    structs::JointType jnt_type = structs::JointType(model.jnt_type[jnt_id]);
    data.jnt_axis[jnt_id].setZero();
    uint16_t q_addr = model.jnt_qaddr[jnt_id];
    if (jnt_type == structs::REVOLUTE) {
      double q_i = data.q[q_addr];
      Eigen::Matrix3d jnt_rel_rot =
          spatial::axisangle2rot(q_i * model.jnt_axis_local[jnt_id].tail(3));
      data.jnt_rot[jnt_id] = data.jnt_rot[jnt_id] * jnt_rel_rot;
      data.jnt_axis[jnt_id].tail(3) =
          data.jnt_rot[jnt_id] * model.jnt_axis_local[jnt_id].tail(3);
    } else if (jnt_type == structs::PRISMATIC) {
      data.jnt_axis[jnt_id].head(3) =
          data.jnt_rot[jnt_id] * model.jnt_axis_local[jnt_id].head(3);
      double q_i = data.q[q_addr];
      data.jnt_pos[jnt_id] +=
          data.jnt_axis[jnt_id].head(3) * q_i; // Update position
    } else if (jnt_type == structs::FREE) {
      auto q_floating = data.q(Eigen::seqN(q_addr, 7));
      data.jnt_axis[jnt_id].head(3) =
          data.jnt_rot[jnt_id] * model.jnt_axis_local[jnt_id].head(3);
      data.jnt_axis[jnt_id].tail(3) =
          data.jnt_rot[jnt_id] * model.jnt_axis_local[jnt_id].tail(3);

      data.jnt_pos[jnt_id] += q_floating.head(3); // Update position
      data.jnt_rot[jnt_id] *= spatial::quat2rot(q_floating.tail(4));
    } else if (jnt_type == structs::BALL) {
      auto q_floating = data.q(Eigen::seqN(q_addr, 4));
      data.jnt_axis[jnt_id].tail(3) =
          data.jnt_rot[jnt_id] * model.jnt_axis_local[jnt_id].tail(3);

      data.jnt_rot[jnt_id] *= spatial::quat2rot(q_floating);
    } else if (jnt_type != structs::FIXED) {
      // Print error that joint is unsupported
      std::cerr << "Joint type not supported: " << jnt_type << std::endl;
    }

    // Set child body position and rotation
    data.link_i_pos[child_id] =
        data.jnt_pos[jnt_id] +
        data.jnt_rot[jnt_id] * model.link_i_pos[child_id];

    data.link_i_rot[child_id] =
        model.link_i_rot[child_id] * data.jnt_rot[jnt_id];
  };
}

inline void computeCompositeMassInertia(const dyn::structs::Model &model,
                                        dyn::structs::Data &data) {
  // Set the subcom mass and inertia to zero
  data.link_subtree_mass = model.link_mass;
  uint16_t child_link_id, parent_link_id;
  for (uint16_t i = 0; i < model.nl; ++i) {
    data.link_subtree_com[i] = data.link_i_pos[i] * model.link_mass[i];

    auto dR = data.link_i_rot[i] * model.link_i_rot[i];
    // data.link_I_w -- orientation in world, point -- link CoM
    data.link_I_w[i] = dR * model.link_I[i] * dR.transpose();
  }

  for (int16_t jnt_id = model.nj - 1; jnt_id >= 0; --jnt_id) {
    child_link_id = model.jnt_childid[jnt_id];
    parent_link_id = model.jnt_parentid[jnt_id];
    data.link_subtree_com[parent_link_id] +=
        data.link_subtree_com[child_link_id];
    data.link_subtree_mass[parent_link_id] +=
        data.link_subtree_mass[child_link_id];
  }

  for (uint16_t i = 0; i < model.nl; ++i) {
    data.link_subtree_com[i] /= data.link_subtree_mass[i];
    data.link_subtree_I[i] = spatial::move_I(
        data.link_I_w[i], data.link_subtree_com[i] - data.link_i_pos[i],
        model.link_mass[i]);
  }
  for (int16_t jnt_id = model.nj - 1; jnt_id >= 0; --jnt_id) {
    child_link_id = model.jnt_childid[jnt_id];
    parent_link_id = model.jnt_parentid[jnt_id];
    data.link_subtree_I[parent_link_id] +=
        spatial::move_I(data.link_subtree_I[child_link_id],
                        data.link_subtree_com[parent_link_id] -
                            data.link_subtree_com[child_link_id],
                        data.link_subtree_mass[child_link_id]);
  }
}

inline void computeMassMatrix(const dyn::structs::Model &model,
                              dyn::structs::Data &data) {
  Eigen::Matrix<double, 6, 6> I_c;
  Eigen::Vector<double, 6> F;

  for (int16_t i = model.nl - 1; i > 0; --i) {
    uint16_t jnt_id = model.link_parentid[i];
    if (model.jnt_type[jnt_id] == structs::FIXED) {
      continue; // Skip fixed joints
    }
    uint16_t dof_adr = model.jnt_dofadr[jnt_id];

    I_c = spatial::construct_spatial_inertia(
        data.link_subtree_mass[i], data.link_subtree_I[i],
        data.link_subtree_com[i] - data.jnt_pos[jnt_id]);
    if (model.jnt_type[jnt_id] == structs::FREE) {
      data.M.block<6, 6>(dof_adr, dof_adr) = I_c;
      continue;
    }
    F = I_c * data.jnt_axis[jnt_id];
    data.M(dof_adr, dof_adr) += data.jnt_axis[jnt_id].dot(F);

    uint16_t p_link_id = i;
    uint16_t p_link_id_next = model.jnt_parentid[jnt_id];
    while (p_link_id_next != 0) {
      uint16_t jnt_id_next = model.link_parentid[p_link_id_next];

      F = spatial::get_dof_mapping_matrix(data.jnt_pos[jnt_id_next] -
                                          data.jnt_pos[jnt_id]) *
          F;

      p_link_id = p_link_id_next;
      jnt_id = jnt_id_next;
      int16_t other_dof = model.jnt_dofadr[jnt_id];
      if (model.jnt_type[jnt_id] == structs::FREE) {
        data.M.block<1, 6>(dof_adr, other_dof) += F;
        data.M.block<6, 1>(other_dof, dof_adr) += F.transpose();
      } else if (model.jnt_type[jnt_id] != structs::FIXED) {
        data.M(dof_adr, other_dof) = F.dot(data.jnt_axis[jnt_id]);
        data.M(other_dof, dof_adr) = data.M(dof_adr, other_dof);
      }

      p_link_id = p_link_id_next;
      p_link_id_next = model.jnt_parentid[jnt_id];
    }
  }
}
} // namespace kinematics
} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_KINEMATICS_HPP