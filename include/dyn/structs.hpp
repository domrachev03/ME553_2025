#ifndef DYN_STRUCTS_HPP
#define DYN_STRUCTS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
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
  std::vector<uint16_t> jnt_dofadr;

  std::vector<uint16_t> qpos_jnt_id;
  std::vector<uint16_t> dof_jnt_id;
};

struct Data {
  Eigen::VectorXd q;
  Eigen::VectorXd v;

  std::vector<Eigen::Vector3d> link_i_pos;
  std::vector<Eigen::Matrix3d> link_i_rot;
  std::vector<Eigen::Vector3d> link_lvel;
  std::vector<Eigen::Vector3d> link_avel;
  std::vector<Eigen::Matrix3d> link_I_w;
  std::vector<double> link_subtree_mass;
  std::vector<Eigen::Vector3d> link_subtree_com;
  std::vector<Eigen::Matrix3d> link_subtree_I;
  std::vector<Eigen::Vector3d> link_ext_force;
  std::vector<Eigen::Vector3d> link_ext_torque;

  std::vector<Eigen::Vector3d> jnt_pos;
  std::vector<Eigen::Matrix3d> jnt_rot;
  std::vector<Eigen::Vector3d> jnt_lvel;
  std::vector<Eigen::Vector3d> jnt_avel;
  // This is axis along which the joint is moving in the world frame
  // First three components are translation, last three are rotation
  std::vector<Eigen::Vector<double, 6>> jnt_axis;

  Eigen::MatrixXd M;
  Eigen::VectorXd b;
  Eigen::Vector3d gravity;
};

inline structs::Data makeData(const structs::Model& model) {
  structs::Data data;
  // zero‐init q and v
  data.q = Eigen::VectorXd::Zero(model.nq);
  data.v = Eigen::VectorXd::Zero(model.nv);

  // zero‐init link quantities
  data.link_i_pos.assign(model.nl, Eigen::Vector3d::Zero());
  data.link_i_rot.assign(model.nl, Eigen::Matrix3d::Zero());
  data.link_lvel.assign(model.nl, Eigen::Vector3d::Zero());
  data.link_avel.assign(model.nl, Eigen::Vector3d::Zero());
  data.link_I_w.assign(model.nl, Eigen::Matrix3d::Zero());
  data.link_subtree_mass.assign(model.nl, 0.0);
  data.link_subtree_com.assign(model.nl, Eigen::Vector3d::Zero());
  data.link_subtree_I.assign(model.nl, Eigen::Matrix3d::Zero());
  data.link_ext_force.assign(model.nl, Eigen::Vector3d::Zero());
  data.link_ext_torque.assign(model.nl, Eigen::Vector3d::Zero());

  // zero‐init joint quantities
  data.jnt_pos.assign(model.nj, Eigen::Vector3d::Zero());
  data.jnt_rot.assign(model.nj, Eigen::Matrix3d::Zero());
  data.jnt_lvel.assign(model.nj, Eigen::Vector3d::Zero());
  data.jnt_avel.assign(model.nj, Eigen::Vector3d::Zero());
  data.jnt_axis.assign(model.nj, Eigen::Matrix<double,6,1>::Zero());

  // zero‐init mass matrix, bias vector and gravity
  data.M   = Eigen::MatrixXd::Zero(model.nv, model.nv);
  data.b   = Eigen::VectorXd::Zero(model.nv);
  data.gravity.setZero();

  return data;
}
} // namespace structs

} // namespace dyn
#endif // DYN_STRUCTS_HPP