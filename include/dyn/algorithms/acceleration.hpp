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

inline std::pair<std::vector<Eigen::Vector<double, 6>>,
                 std::vector<Eigen::Vector<double, 6>>>
computeAcceleration(const dyn::structs::Model &model,
                    const dyn::structs::Data &data, const Eigen::VectorXd &dv) {

  auto link_acc = std::vector<Eigen::Vector<double, 6>>(model.nl);

  auto jnt_acc = std::vector<Eigen::Vector<double, 6>>(model.nj);

  link_acc[0] = Eigen::Vector<double, 6>::Zero();
  jnt_acc[0] = Eigen::Vector<double, 6>::Zero();

  structs::JointType jnt_type;
  uint16_t q_addr, dof_addr, parent_link_id, parent_jnt_id;
  Eigen::Matrix3d omega_skew, alpha_skew;

  for (uint16_t jnt_id = 0; jnt_id < model.nj; ++jnt_id) {
    q_addr = model.jnt_qaddr[jnt_id];
    dof_addr = model.jnt_dofadr[jnt_id];
    parent_link_id = model.jnt_parentid[jnt_id];
    parent_jnt_id = model.link_parentid[parent_link_id];
    if (parent_jnt_id == UINT16_MAX) {
      parent_jnt_id = 0; // Set to base if no parent
    }

    omega_skew = spatial::skew_matrix(data.jnt_avel[parent_jnt_id]);
    alpha_skew = spatial::skew_matrix(jnt_acc[parent_jnt_id].tail(3));

    // First, compute link accelerations of the parent link
    link_acc[parent_link_id] = jnt_acc[parent_jnt_id];
    link_acc[parent_link_id].head(3) +=
        alpha_skew * data.jnt_rot[parent_jnt_id] *
            model.link_i_pos[parent_link_id] +
        omega_skew * omega_skew * data.jnt_rot[parent_jnt_id] *
            model.link_i_pos[parent_link_id];

    // Then, compute joint acceleration of the current joint
    jnt_acc[jnt_id] = jnt_acc[parent_jnt_id];
    jnt_acc[jnt_id].head(3) +=
        alpha_skew * data.jnt_rot[parent_jnt_id] * model.jnt_rel_pos[jnt_id] +
        omega_skew * omega_skew * data.jnt_rot[parent_jnt_id] *
            model.jnt_rel_pos[jnt_id];

    // Type-dependent components of the joint acceleration
    jnt_type = structs::JointType(model.jnt_type[jnt_id]);
    if (jnt_type == structs::PRISMATIC) {
      // TODO: complete
    } else if (jnt_type == structs::REVOLUTE) {
      jnt_acc[jnt_id].tail(3) += data.jnt_axis[jnt_id].tail(3) * dv[dof_addr] +
                                 spatial::skew_matrix(data.jnt_avel[jnt_id]) *
                                     data.jnt_axis[jnt_id].tail(3) *
                                     data.v[dof_addr];
    } else if (jnt_type == structs::FREE) {
      jnt_acc[jnt_id] += dv(Eigen::seqN(q_addr, 6));
    }
  }

  return std::make_pair(link_acc, jnt_acc);
}

} // namespace acceleration
} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_ACCELERATION_HPP