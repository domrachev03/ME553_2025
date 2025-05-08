#ifndef DYN_ALGORITHMS_KINEMATICS_HPP
#define DYN_ALGORITHMS_KINEMATICS_HPP

#include "../spatial.hpp" // Changed from <dyn/spatial.hpp>
#include "../structs.hpp" // Changed from <dyn/structs.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <iostream> // For std::cerr, std::endl
#include <vector> // For Eigen::seqN if used with std::vector indirectly, or for other vector uses

namespace dyn {
namespace algorithms {

namespace kinematics {

inline void computeForwardKinematics(const dyn::structs::Model &model,
                                     dyn::structs::Data &data) {
  data.link_pos[0] = Eigen::Vector3d::Zero();
  data.link_rot[0] = Eigen::Matrix3d::Identity();
  for (uint16_t jnt_id = 0; jnt_id < model.nj; ++jnt_id) {
    // Get the parent frame
    uint16_t parent_id = model.jnt_parentid[jnt_id];
    uint16_t child_id = model.jnt_childid[jnt_id];
    Eigen::Vector3d parent_pos = data.link_pos[parent_id];
    Eigen::Matrix3d parent_rot = data.link_rot[parent_id];

    // Compute the forward kinematics for each joint
    const auto &jnt_pos = model.jnt_rel_pos[jnt_id];
    const auto &joint_rot = model.jnt_rel_rot[jnt_id];

    data.jnt_pos[jnt_id] = parent_pos + parent_rot * jnt_pos;
    data.jnt_rot[jnt_id] = parent_rot * joint_rot;

    // Handle joint effect
    structs::JointType jnt_type = structs::JointType(model.jnt_type[jnt_id]);
    data.jnt_axis_pos[jnt_id] = Eigen::Vector3d::Zero();
    uint16_t q_addr = model.jnt_qaddr[jnt_id];
    if (jnt_type == structs::REVOLUTE) {
      double q_i = data.q[q_addr];
      Eigen::Matrix3d jnt_rel_rot =
          spatial::axisangle2rot(q_i * model.jnt_axis_local[jnt_id].tail(3));
      data.jnt_rot[jnt_id] = data.jnt_rot[jnt_id] * jnt_rel_rot;
      data.jnt_axis_rot[jnt_id] =
          data.jnt_rot[jnt_id] * model.jnt_axis_local[jnt_id].tail(3);
    } else if (jnt_type == structs::PRISMATIC) {
      data.jnt_axis_pos[jnt_id] =
          data.jnt_rot[jnt_id] * model.jnt_axis_local[jnt_id].head(3);
      double q_i = data.q[q_addr];
      data.jnt_pos[jnt_id] +=
          data.jnt_axis_pos[jnt_id] * q_i; // Update position
    } else if (jnt_type == structs::FREE) {
      auto q_floating = data.q(Eigen::seqN(q_addr, 7));
      data.jnt_axis_pos[jnt_id] =
          data.jnt_rot[jnt_id] * model.jnt_axis_local[jnt_id].head(3);
      data.jnt_axis_rot[jnt_id] =
          data.jnt_rot[jnt_id] * model.jnt_axis_local[jnt_id].tail(3);

      data.jnt_pos[jnt_id] += q_floating.head(3); // Update position
      data.jnt_rot[jnt_id] *= spatial::quat2rot(q_floating.tail(4));

    } else if (jnt_type != structs::FIXED) {
      // Print error that joint is unsupported
      std::cerr << "Joint type not supported: " << jnt_type << std::endl;
    }

    // Set child body position and rotation
    data.link_pos[child_id] = data.jnt_pos[jnt_id];
    data.link_rot[child_id] = data.jnt_rot[jnt_id];
  };
}
} // namespace kinematics

} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_KINEMATICS_HPP