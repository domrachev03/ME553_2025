#ifndef DYN_ALGORITHMS_ACCELERATION_HPP
#define DYN_ALGORITHMS_ACCELERATION_HPP

#include "../spatial.hpp"
#include "../structs.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

namespace dyn {
namespace algorithms {

namespace acceleration {

inline void computeAcceleration(const dyn::structs::Model &model,
                                dyn::structs::Data &data) {
  data.jnt_lacc[0] = Eigen::Vector3d::Zero();
  data.jnt_aacc[0] = Eigen::Vector3d::Zero();

  structs::JointType jnt_type;
  for (uint16_t jnt_id = 0; jnt_id < model.nj; ++jnt_id) {
    uint16_t q_addr = model.jnt_qaddr[jnt_id];
    uint16_t dof_addr = model.jnt_dofadr[jnt_id];
    uint16_t parent_link_id = model.jnt_parentid[jnt_id];
    uint16_t parent_jnt_id = model.link_parentid[parent_link_id];
    if (parent_jnt_id == UINT16_MAX) {
      parent_jnt_id = 0; // Set to base if no parent
    }

    auto omega_skew = spatial::skew_matrix(data.jnt_avel[parent_jnt_id]);
    auto alpha_skew = spatial::skew_matrix(data.jnt_aacc[parent_jnt_id]);

    // First, compute link accelerations of the parent link
    data.link_lacc[parent_link_id] = data.jnt_lacc[parent_jnt_id] +
                                     alpha_skew * data.jnt_rot[parent_jnt_id] *
                                         model.link_i_pos[parent_link_id] +
                                     omega_skew * omega_skew *
                                         data.jnt_rot[parent_jnt_id] *
                                         model.link_i_pos[parent_link_id];
    data.link_aacc[parent_link_id] = data.jnt_aacc[parent_jnt_id];

    // Then, compute joint acceleration of the current joint
    data.jnt_lacc[jnt_id] =
        data.jnt_lacc[parent_jnt_id] +
        alpha_skew * data.jnt_rot[parent_jnt_id] * model.jnt_rel_pos[jnt_id] +
        omega_skew * omega_skew * data.jnt_rot[parent_jnt_id] *
            model.jnt_rel_pos[jnt_id];
    data.jnt_aacc[jnt_id] = data.jnt_aacc[parent_jnt_id];

    // Type-dependent components of the joint acceleration
    jnt_type = structs::JointType(model.jnt_type[jnt_id]);
    if (jnt_type == structs::PRISMATIC) {
      // data.jnt_lvel[jnt_id] += 0;
    } else if (jnt_type == structs::REVOLUTE) {
      data.jnt_aacc[jnt_id] +=
          data.jnt_axis[jnt_id].tail(3) * data.dv[dof_addr] +
          spatial::skew_matrix(data.jnt_avel[jnt_id]) *
              data.jnt_axis[jnt_id].tail(3) * data.v[dof_addr];
    } else if (jnt_type == structs::FREE) {
      data.jnt_lacc[jnt_id] += data.dv(Eigen::seqN(q_addr, 3));
      data.jnt_aacc[jnt_id] += data.dv(Eigen::seqN(q_addr + 3, 3));
    }
  }
}

} // namespace acceleration
} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_ACCELERATION_HPP