#ifndef DYN_STRUCTS_HPP
#define DYN_STRUCTS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <arm_neon.h>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <tinyxml_rai/tinystr.h>
#include <tinyxml_rai/tinyxml_rai.h>
#include <vector>

namespace dyn {
namespace structs {
enum JointType {
  FIXED = 0,
  REVOLUTE = 1,
  PRISMATIC = 2,
  FREE = 3,
  BALL = 4,
};

inline JointType getJointType(const std::string &type) {
  if (type == "fixed") {
    return FIXED;
  } else if (type == "revolute" || type == "continuous") {
    return REVOLUTE;
  } else if (type == "prismatic") {
    return PRISMATIC;
  } else if (type == "free") {
    return FREE;
  } else {
    throw std::runtime_error("Unknown joint type: " + type);
  }
}

inline uint16_t getJointDof(JointType type) {
  switch (type) {
  case FIXED:
    return 0;
  case REVOLUTE:
    return 1;
  case PRISMATIC:
    return 1;
  case FREE:
    return 6;
  default:
    throw std::runtime_error("Unknown joint type");
  }
}
inline uint16_t getJointQdof(JointType type) {
  switch (type) {
  case FIXED:
    return 0;
  case REVOLUTE:
    return 1;
  case PRISMATIC:
    return 1;
  case FREE:
    return 7;
  default:
    throw std::runtime_error("Unknown joint type");
  }
}

struct Model {
  uint16_t nl;
  uint16_t nj;
  uint16_t nq;
  uint16_t nv;
  std::vector<std::string> link_name;
  std::vector<uint16_t> link_parentid;
  std::vector<Eigen::Vector3d> link_i_pos;
  std::vector<Eigen::Matrix3d> link_i_rot;
  std::vector<double> link_mass;
  std::vector<Eigen::Matrix3d> link_I;

  std::vector<std::string> jnt_name;
  std::vector<uint16_t> jnt_parentid;
  std::vector<uint16_t> jnt_childid;
  std::vector<Eigen::Vector3d> jnt_rel_pos;
  std::vector<Eigen::Matrix3d> jnt_rel_rot;
  std::vector<JointType> jnt_type;
  std::vector<Eigen::Vector<double, 6>> jnt_axis_local;
  std::vector<Eigen::Vector2d> jnt_range;
  std::vector<uint16_t> jnt_qaddr;
  Eigen::VectorX<uint16_t> jnt_dofadr;

  std::vector<uint16_t> qpos_jnt_id;
  Eigen::VectorX<uint16_t> dof_jnt_id;
};

struct Data {
  Eigen::VectorXd q;
  Eigen::VectorXd v;

  std::vector<Eigen::Vector3d> link_i_pos;
  std::vector<Eigen::Matrix3d> link_i_rot;
  std::vector<Eigen::Vector3d> link_I_world;
  std::vector<Eigen::Vector3d> link_lvel;
  std::vector<Eigen::Vector3d> link_avel;
  std::vector<Eigen::MatrixXd> link_Jpos;
  std::vector<Eigen::MatrixXd> link_Jrot;
  std::vector<Eigen::Vector3d> link_subtree_com;
  std::vector<Eigen::Matrix3d> link_subtree_I;

  std::vector<Eigen::MatrixXd> jnt_Jpos;
  std::vector<Eigen::MatrixXd> jnt_Jrot;
  std::vector<Eigen::Vector3d> jnt_pos;
  std::vector<Eigen::Matrix3d> jnt_rot;
  std::vector<Eigen::Vector3d> jnt_lvel;
  std::vector<Eigen::Vector3d> jnt_avel;
  // This is axis along which the joint is moving in the world frame
  // First three components are translation, last three are rotation
  std::vector<Eigen::Vector3d> jnt_axis_pos;
  std::vector<Eigen::Vector3d> jnt_axis_rot;
};

inline structs::Data makeData(const structs::Model &model) {
  structs::Data data;
  data.q.resize(model.nq);
  data.link_i_pos.resize(model.nl);
  data.link_i_rot.resize(model.nl);
  data.jnt_pos.resize(model.nj);
  data.jnt_rot.resize(model.nj);
  data.jnt_lvel.resize(model.nj);
  data.jnt_avel.resize(model.nj);
  data.jnt_axis_pos.resize(model.nj);
  data.jnt_axis_rot.resize(model.nj);
  data.link_lvel.resize(model.nl);
  data.link_avel.resize(model.nl);
  data.link_Jpos.resize(model.nl);
  data.link_subtree_com.resize(model.nl);
  data.link_subtree_I.resize(model.nl);
  for (uint16_t i = 0; i < model.nl; ++i) {
    data.link_Jpos[i].resize(3, model.nv);
  }
  data.link_Jrot.resize(model.nl);
  for (uint16_t i = 0; i < model.nl; ++i) {
    data.link_Jrot[i].resize(3, model.nv);
  }
  data.jnt_Jpos.resize(model.nj);
  for (uint16_t i = 0; i < model.nj; ++i) {
    data.jnt_Jpos[i].resize(3, model.nv);
  }
  data.jnt_Jrot.resize(model.nj);
  for (uint16_t i = 0; i < model.nj; ++i) {
    data.jnt_Jrot[i].resize(3, model.nv);
  }

  return data;
}
} // namespace structs

} // namespace dyn
#endif // DYN_STRUCTS_HPP