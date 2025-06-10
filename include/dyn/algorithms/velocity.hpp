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

namespace velocity {

inline void computeVelocity(const dyn::structs::Model &model,
                            dyn::structs::Data &data) {
  data.jnt_lvel[0] = Eigen::Vector3d::Zero();
  data.jnt_avel[0] = Eigen::Vector3d::Zero();

  structs::JointType jnt_type;
  Eigen::Matrix3d R_dot;
  uint16_t q_addr, dof_addr, parent_link_id, parent_jnt_id;
  for (uint16_t jnt_id = 0; jnt_id < model.nj; ++jnt_id) {
    // Defining related indices
    q_addr = model.jnt_qaddr[jnt_id];
    dof_addr = model.jnt_dofadr[jnt_id];
    parent_link_id = model.jnt_parentid[jnt_id];
    parent_jnt_id = model.link_parentid[parent_link_id];
    if (parent_jnt_id == UINT16_MAX) {
      parent_jnt_id = 0; // Set to base if no parent
    }
    // First, compute link velocities of the parent link
    R_dot = spatial::skew_matrix(data.jnt_avel[parent_jnt_id]) *
            data.jnt_rot[parent_jnt_id];
    data.link_lvel[parent_link_id] =
        data.jnt_lvel[parent_jnt_id] + R_dot * model.link_i_pos[parent_link_id];
    data.link_avel[parent_link_id] = data.jnt_avel[parent_jnt_id];

    // Then, compute joint velocity of the current joint
    // Common components of the joint velocity
    data.jnt_lvel[jnt_id] =
        data.jnt_lvel[parent_jnt_id] + R_dot * model.jnt_rel_pos[jnt_id];
    data.jnt_avel[jnt_id] = data.jnt_avel[parent_jnt_id];

    // Type-dependent components of the joint velocity
    jnt_type = structs::JointType(model.jnt_type[jnt_id]);
    if (jnt_type == structs::PRISMATIC) {
      data.jnt_lvel[jnt_id] += spatial::skew_matrix(data.jnt_avel[jnt_id]) *
                                   data.jnt_axis[jnt_id].head(3) *
                                   data.q[q_addr] +
                               data.jnt_axis[jnt_id].head(3) * data.v[dof_addr];
    } else if (jnt_type == structs::REVOLUTE) {
      data.jnt_avel[jnt_id] += data.jnt_axis[jnt_id].tail(3) * data.v[dof_addr];
    } else if (jnt_type == structs::FREE) {
      data.jnt_lvel[jnt_id] += data.v(Eigen::seqN(dof_addr, 3));
      data.jnt_avel[jnt_id] += data.v(Eigen::seqN(dof_addr + 3, 3));
    } else if (jnt_type == structs::BALL) {
      data.jnt_avel[jnt_id] +=
          data.jnt_rot[jnt_id] * data.v(Eigen::seqN(dof_addr, 3));
    }
  }
}

} // namespace velocity
} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_JACOBIAN_HPP