#ifndef DYN
#define DYN

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <arm_neon.h>
#include <cstdint>
#include <dyn/structs.hpp>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <tinyxml_rai/tinystr.h>
#include <tinyxml_rai/tinyxml_rai.h>

namespace dyn {
namespace utils {

inline void printModelInfo(const dyn::structs::Model &model) {
  std::cout << "Number of links: " << model.nl << std::endl;
  std::cout << "Number of joints: " << model.nj << std::endl;
  std::cout << "Number of qpos: " << model.nq << std::endl;
  std::cout << "Link names: ";
  // Print each link and its name on a separate line
  for (uint16_t i = 0; i < model.nl; ++i) {
    std::cout << "Link Name: " << model.link_name[i] << std::endl;

    // Link inertia
    std::cout << "  Mass: " << model.link_mass[i] << std::endl;
    std::cout << "  Inertia: " << std::endl;
    std::cout << "    [" << model.link_I[i](0, 0) << ", "
              << model.link_I[i](0, 1) << ", " << model.link_I[i](0, 2) << "]"
              << std::endl;
    std::cout << "    [" << model.link_I[i](1, 0) << ", "
              << model.link_I[i](1, 1) << ", " << model.link_I[i](1, 2) << "]"
              << std::endl;
    std::cout << "    [" << model.link_I[i](2, 0) << ", "
              << model.link_I[i](2, 1) << ", " << model.link_I[i](2, 2) << "]"
              << std::endl;
    std::cout << "  Position: [" << model.link_i_pos[i][0] << ", "
              << model.link_i_pos[i][1] << ", " << model.link_i_pos[i][2] << "]"
              << std::endl;
    std::cout << "  Rotation:" << std::endl;
    std::cout << "    [" << model.link_i_rot[i](0, 0) << ", "
              << model.link_i_rot[i](0, 1) << ", " << model.link_i_rot[i](0, 2)
              << "]" << std::endl;
    std::cout << "    [" << model.link_i_rot[i](1, 0) << ", "
              << model.link_i_rot[i](1, 1) << ", " << model.link_i_rot[i](1, 2)
              << "]" << std::endl;
    std::cout << "    [" << model.link_i_rot[i](2, 0) << ", "
              << model.link_i_rot[i](2, 1) << ", " << model.link_i_rot[i](2, 2)
              << "]" << std::endl;
  }
  std::cout << std::endl;

  // Print each joint's information on separate lines
  for (uint16_t i = 0; i < model.nj; ++i) {
    std::cout << "Joint Name: " << model.jnt_name[i] << std::endl;
    std::cout << "  Type: " << model.jnt_type[i] << std::endl;
    std::cout << "  Range: [" << model.jnt_range[i][0] << ", "
              << model.jnt_range[i][1] << "]" << std::endl;
    std::cout << "  Parent Link: " << model.link_name[model.jnt_parentid[i]]
              << std::endl;
    std::cout << "  Child Link: " << model.link_name[model.jnt_childid[i]]
              << std::endl;
    std::cout << "  Relative Position: [" << model.jnt_rel_pos[i][0] << ", "
              << model.jnt_rel_pos[i][1] << ", " << model.jnt_rel_pos[i][2]
              << "]" << std::endl;
    std::cout << "  Relative Rotation:" << std::endl;
    std::cout << "    [" << model.jnt_rel_rot[i](0, 0) << ", "
              << model.jnt_rel_rot[i](0, 1) << ", "
              << model.jnt_rel_rot[i](0, 2) << "]" << std::endl;
    std::cout << "    [" << model.jnt_rel_rot[i](1, 0) << ", "
              << model.jnt_rel_rot[i](1, 1) << ", "
              << model.jnt_rel_rot[i](1, 2) << "]" << std::endl;
    std::cout << "    [" << model.jnt_rel_rot[i](2, 0) << ", "
              << model.jnt_rel_rot[i](2, 1) << ", "
              << model.jnt_rel_rot[i](2, 2) << "]" << std::endl;
    std::cout << "  Axis (6D): [";
    for (int k = 0; k < 6; ++k) {
      std::cout << model.jnt_axis_local[i][k] << (k + 1 < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    std::cout << "  qaddr: " << model.jnt_qaddr[i] << std::endl;
    std::cout << "---------------------------------------" << std::endl;
  }

  // Print qpos joint IDs for each joint
  std::cout << "Joint starting qpos IDs:" << std::endl;
  uint16_t qpos_index = 0;
  for (uint16_t i = 0; i < model.nj; ++i) {
    for (uint16_t j = 0; j < structs::getJointQdof(model.jnt_type[i]); ++j) {
      std::cout << "Joint " << model.jnt_name[i] << ": "
                << model.qpos_jnt_id[qpos_index] << std::endl;
      qpos_index++;
    }
  }
}

inline uint16_t jnt_name2id(const dyn::structs::Model &model,
                            std::string jnt_name) {
  return std::find(model.jnt_name.begin(), model.jnt_name.end(), jnt_name) -
         model.jnt_name.begin();
}

inline uint16_t link_name2id(const dyn::structs::Model &model,
                             std::string link_name) {
  return std::find(model.link_name.begin(), model.link_name.end(), link_name) -
         model.link_name.begin();
}

template <typename T>
inline void reorder(std::vector<T> &vec, const std::vector<uint16_t> &order) {
  for (int s = 1, d; s < order.size(); ++s) {
    for (d = order[s]; d < s; d = order[d])
      ;
    if (d == s)
      while (d = order[d], d != s)
        std::swap(vec[s], vec[d]);
  }
}
} // namespace utils
} // namespace dyn
#endif // DYN