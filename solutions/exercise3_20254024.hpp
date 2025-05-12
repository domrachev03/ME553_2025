#pragma once
#include <Eigen/Dense>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stack>
#include <stdexcept>
#include <string>
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
  std::vector<uint16_t> dof_jnt_id;
};

struct Data {
  Eigen::VectorXd q;
  Eigen::VectorXd v;

  std::vector<Eigen::Vector3d> link_i_pos;
  std::vector<Eigen::Matrix3d> link_i_rot;
  std::vector<Eigen::Vector3d> link_lvel;
  std::vector<Eigen::Vector3d> link_avel;
  std::vector<Eigen::MatrixXd> link_Jpos;
  std::vector<Eigen::MatrixXd> link_Jrot;
  std::vector<Eigen::Matrix3d> link_I_w;
  std::vector<double> link_subtree_mass;
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
  std::vector<Eigen::Vector<double, 6>> jnt_axis;

  Eigen::MatrixXd M;
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
  data.jnt_axis.resize(model.nj);
  data.link_lvel.resize(model.nl);
  data.link_avel.resize(model.nl);
  data.link_Jpos.resize(model.nl);
  data.link_I_w.resize(model.nl);
  data.link_subtree_mass.resize(model.nl);
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

  data.M.resize(model.nv, model.nv);

  return data;
}
} // namespace structs
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
namespace spatial {
inline Eigen::Matrix<double, 3, 3> rpy2rot(const Eigen::Vector3d &rpy) {
  Eigen::Matrix<double, 3, 3> rot;
  double thX = rpy[0], thY = rpy[1], thZ = rpy[2];

  Eigen::Matrix3d xRot, yRot, zRot;

  xRot << 1, 0, 0, 0, cos(thX), -sin(thX), 0, sin(thX), cos(thX);

  yRot << cos(thY), 0, sin(thY), 0, 1, 0, -sin(thY), 0, cos(thY);

  zRot << cos(thZ), -sin(thZ), 0, sin(thZ), cos(thZ), 0, 0, 0, 1;

  rot = xRot * yRot * zRot;
  return rot;
}

inline Eigen::Matrix3d axisangle2rot(const Eigen::Vector3d &axis) {
  Eigen::Matrix3d rot;
  double theta = axis.norm();
  Eigen::Vector3d n = axis.normalized();

  return Eigen::AngleAxisd(theta, n).toRotationMatrix();
}

inline Eigen::Matrix3d quat2rot(const Eigen::Vector4d &quat) {
  Eigen::Matrix3d rot;
  double w = quat[0], x = quat[1], y = quat[2], z = quat[3];

  rot << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
      2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
      2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);

  return rot;
}

inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
  Eigen::Matrix3d skew;
  skew << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return skew;
}

inline Eigen::Matrix3d move_I(const Eigen::Matrix3d &I,
                              const Eigen::Vector3d &r, const double &m) {
  return (I + m * (r.squaredNorm() * Eigen::Matrix3d::Identity() -
                   r * r.transpose()));
}

inline Eigen::Matrix<double, 6, 6>
get_dof_mapping_matrix(const Eigen::Vector3d &r) {
  Eigen::Matrix<double, 6, 6> X;
  X.setZero();
  X.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  X.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  X.block<3, 3>(3, 0) = -spatial::skew_matrix(r);

  return X;
}

inline Eigen::Matrix<double, 6, 6>
construct_spatial_inertia(const double &m, const Eigen::Matrix3d &I,
                          const Eigen::Vector3d &r) {
  Eigen::Matrix3d r_skew = spatial::skew_matrix(r);
  Eigen::Matrix3d m_r_skew = m * r_skew;
  Eigen::Matrix<double, 6, 6> J;
  J.setZero();
  J.block<3, 3>(0, 0) = m * Eigen::Matrix3d::Identity();
  J.block<3, 3>(3, 3) = I - m_r_skew * r_skew;
  J.block<3, 3>(0, 3) = -m_r_skew;
  J.block<3, 3>(3, 0) = m_r_skew;

  return J;
}
} // namespace spatial
namespace parse {

inline void setFloatingBase(structs::Model &model) {
  model.jnt_type[0] = structs::FREE;
  // There is no reasonable way to limit ball and floating joints, and only
  // scalar joints have limits
  model.jnt_range[0] = Eigen::Vector2d(0, 0);
  model.jnt_rel_pos[0] = Eigen::Vector3d::Zero();
  model.jnt_rel_rot[0] = Eigen::Matrix3d::Identity();
  model.jnt_qaddr[0] = 0;
  model.jnt_dofadr[0] = 0;
  model.jnt_axis_local[0] = Eigen::Vector<double, 6>::Ones();

  model.link_i_pos[0] = Eigen::Vector3d::Zero();
  model.link_i_rot[0] = Eigen::Matrix3d::Identity();
  model.link_mass[0] = 0.0;
  model.link_I[0] = Eigen::Matrix3d::Zero();

  // First 7 elements of qpos_jnt_id and 6 elements of dof_jnt_id are zeros
  for (uint16_t i = 0; i < 6; ++i) {
    model.qpos_jnt_id[i] = 0;
    model.dof_jnt_id[i] = 0;
  }
  model.qpos_jnt_id[6] = 0;
}

inline structs::Model parseURDF(const std::string &urdf,
                                const bool &floating_base) {
  structs::Model model;
  raisim::TiXmlDocument doc;

  doc.Parse(urdf.c_str());
  raisim::TiXmlElement *root = doc.RootElement();
  if (root == nullptr) {
    throw std::runtime_error("Failed to parse URDF");
  }

  // Parse the number of bodies and joints
  model.nl = 0;
  model.nj = 0;
  model.nq = 0;
  model.nv = 0;

  std::vector<std::string> link_names;
  std::vector<std::string> jnt_names;
  std::vector<std::string> jnt_parent_names;
  std::vector<std::string> jnt_child_names;

  if (floating_base) {
    model.nl = 1;
    model.nj = 1;
    model.nq = 7;
    model.nv = 6;

    link_names.push_back("_root");
    jnt_names.push_back("floating_base_joint");
    jnt_parent_names.push_back("_root");
    jnt_child_names.push_back(""); // It is handled separately later
  }
  // Count bodies and joints, and store their names and parent-child
  // relationships
  for (raisim::TiXmlElement *child = root->FirstChildElement();
       child != nullptr; child = child->NextSiblingElement()) {
    if (strcmp(child->Value(), "link") == 0) {
      model.nl++;
      std::string name(child->Attribute("name"));
      link_names.push_back(name);
    } else if (strcmp(child->Value(), "joint") == 0) {
      model.nj++;
      std::string type(child->Attribute("type"));
      if (!type.size()) {
        throw std::runtime_error("Joint type attribute is missing");
      }
      structs::JointType jointType = structs::getJointType(type);
      model.nq += structs::getJointQdof(jointType);
      model.nv += structs::getJointDof(jointType);

      std::string joint_name(child->Attribute("name"));
      if (!joint_name.size()) {
        throw std::runtime_error("Joint name attribute is missing");
      }
      jnt_names.push_back(joint_name);
      for (raisim::TiXmlElement *jnt_child = child->FirstChildElement();
           jnt_child != nullptr; jnt_child = jnt_child->NextSiblingElement()) {
        if (strcmp(jnt_child->Value(), "parent") == 0) {
          std::string parent_name(jnt_child->Attribute("link"));
          jnt_parent_names.push_back(parent_name);
        }
        if (strcmp(jnt_child->Value(), "child") == 0) {
          std::string child_name(jnt_child->Attribute("link"));
          jnt_child_names.push_back(child_name);
        }
      }
    }
  }
  // Convert joint parent and child names to IDs in unordered setting
  std::vector<uint16_t> jnt_parentid;
  std::vector<uint16_t> jnt_childid;
  std::vector<uint16_t> link_parentid;
  std::vector<std::vector<uint16_t>> link_childid;
  jnt_parentid.resize(model.nj);
  jnt_childid.resize(model.nj);
  link_parentid.resize(model.nl);
  std::fill(link_parentid.begin(), link_parentid.end(), UINT16_MAX);
  link_childid.resize(model.nl);

  // Finding childs for all links
  for (uint16_t i = 0; i < jnt_parent_names.size(); ++i) {
    std::string parent_name = jnt_parent_names[i];
    jnt_parentid[i] =
        std::find(link_names.begin(), link_names.end(), parent_name) -
        link_names.begin();
    link_childid[jnt_parentid[i]].push_back(i);
  }

  // Finding parents for all links
  for (uint16_t i = 0; i < jnt_child_names.size(); ++i) {
    if (floating_base && i == 0) {
      continue;
    }
    std::string child_name = jnt_child_names[i];
    jnt_childid[i] =
        std::find(link_names.begin(), link_names.end(), child_name) -
        link_names.begin();
    link_parentid[jnt_childid[i]] = i;
  }

  // If there is floating base, set the child
  if (floating_base) {
    // Find link with no parent, and set it to be the child of the floating base
    for (uint16_t i = 1; i < model.nl; ++i) {
      if (link_parentid[i] == UINT16_MAX) {
        link_parentid[i] = 0;
        jnt_childid[0] = i;
        jnt_child_names[0] = link_names[i];
        break;
      }
    }
  }

  // Do depth-first search to find the id of link and joint
  std::vector<uint16_t> link_id(model.nl, UINT16_MAX);
  std::vector<uint16_t> jnt_id(model.nj, UINT16_MAX);
  std::stack<uint16_t> link_stack;
  // Put all link without parent into the stack
  for (uint16_t i = 0; i < model.nl; ++i) {
    if (link_parentid[i] == UINT16_MAX) {
      link_stack.push(i);
    }
  }
  uint16_t glob_link_idx = 0;
  uint16_t glob_jnt_idx = 0;
  // Continue until all links are processed
  while (!link_stack.empty()) {
    uint16_t link_idx = link_stack.top();
    link_stack.pop();
    // Assign the index to the link
    link_id[link_idx] = glob_link_idx;
    glob_link_idx++;

    // Assign the index to the parent joint, if it has a parent
    uint16_t parent_jnt_idx = link_parentid[link_idx];
    if (parent_jnt_idx != UINT16_MAX) {
      jnt_id[parent_jnt_idx] = glob_jnt_idx;
      glob_jnt_idx++;
    }

    // Push all child joints into the stack
    for (int16_t j = link_childid[link_idx].size() - 1; j >= 0; --j) {
      link_stack.push(jnt_childid[link_childid[link_idx][j]]);
    }
  }
  // Reorder the link and joint names and their parent-child relationships
  utils::reorder(link_names, link_id);
  utils::reorder(jnt_names, jnt_id);
  utils::reorder(jnt_parent_names, jnt_id);
  utils::reorder(jnt_child_names, jnt_id);

  // Get new indices for the parent and child joints
  for (uint16_t i = 0; i < jnt_parent_names.size(); ++i) {
    std::string parent_name = jnt_parent_names[i];
    jnt_parentid[i] =
        std::find(link_names.begin(), link_names.end(), parent_name) -
        link_names.begin();
  }
  for (uint16_t i = 0; i < jnt_child_names.size(); ++i) {
    std::string child_name = jnt_child_names[i];
    jnt_childid[i] =
        std::find(link_names.begin(), link_names.end(), child_name) -
        link_names.begin();
    link_parentid[jnt_childid[i]] = i;
  }
  // Write the reordered names to the model
  model.link_name = link_names;
  model.jnt_name = jnt_names;
  model.link_parentid = link_parentid;
  model.jnt_parentid = jnt_parentid;
  model.jnt_childid = jnt_childid;

  // Initialize vectors
  model.link_i_pos.resize(model.nl);
  model.link_i_rot.resize(model.nl);
  model.link_mass.resize(model.nl);
  model.link_I.resize(model.nl);
  model.jnt_type.resize(model.nj);
  model.jnt_range.resize(model.nj);
  model.jnt_rel_pos.resize(model.nj);
  model.jnt_rel_rot.resize(model.nj);
  model.jnt_qaddr.resize(model.nj);
  model.jnt_axis_local.resize(model.nj);
  model.jnt_dofadr.resize(model.nj);

  model.qpos_jnt_id.resize(model.nq);
  model.dof_jnt_id.resize(model.nv);
  // Fill in the data
  if (floating_base) {
    setFloatingBase(model);
  }
  uint16_t jnt_qdof_index = floating_base ? 7 : 0;
  uint16_t jnt_dof_index = floating_base ? 6 : 0;

  for (raisim::TiXmlElement *child = root->FirstChildElement();
       child != nullptr; child = child->NextSiblingElement()) {
    if (strcmp(child->Value(), "link") == 0) {
      std::string link_name(child->Attribute("name"));
      if (!link_name.size()) {
        throw std::runtime_error("Link name attribute is missing");
      }
      // Getting index of the link
      uint16_t link_index = utils::link_name2id(model, link_name);

      // Setting default values
      model.link_i_pos[link_index] = Eigen::Vector3d::Zero();
      model.link_i_rot[link_index] = Eigen::Matrix3d::Identity();
      model.link_mass[link_index] = 0.0;
      model.link_I[link_index] = Eigen::Matrix3d::Zero();

      for (raisim::TiXmlElement *jnt_child = child->FirstChildElement();
           jnt_child != nullptr; jnt_child = jnt_child->NextSiblingElement()) {
        if (strcmp(jnt_child->Value(), "inertial") == 0) {
          // Parse the inertial <origin> element (xyz and rpy)
          // Iterate over all children of the <inertial> element
          for (raisim::TiXmlElement *elt = jnt_child->FirstChildElement();
               elt != nullptr; elt = elt->NextSiblingElement()) {
            const char *tag = elt->Value();

            if (std::strcmp(tag, "origin") == 0) {
              // parse <origin xyz="..." rpy="...">
              const char *xyz = elt->Attribute("xyz");
              if (xyz) {
                if (std::sscanf(xyz, "%lf %lf %lf",
                                &model.link_i_pos[link_index][0],
                                &model.link_i_pos[link_index][1],
                                &model.link_i_pos[link_index][2]) != 3) {
                  throw std::runtime_error(
                      "Invalid inertial origin xyz for link " + link_name);
                }
              }
              const char *rpy = elt->Attribute("rpy");
              if (rpy) {
                Eigen::Vector3d rpy_vec;
                if (std::sscanf(rpy, "%lf %lf %lf", &rpy_vec[0], &rpy_vec[1],
                                &rpy_vec[2]) != 3) {
                  throw std::runtime_error(
                      "Invalid inertial origin rpy for link " + link_name);
                }
                model.link_i_rot[link_index] = spatial::rpy2rot(rpy_vec);
              }

            } else if (std::strcmp(tag, "mass") == 0) {
              // parse <mass value="...">
              const char *val = elt->Attribute("value");
              if (!val ||
                  std::sscanf(val, "%lf", &model.link_mass[link_index]) != 1) {
                throw std::runtime_error(
                    "Invalid mass attribute in URDF for link " + link_name);
              }

            } else if (std::strcmp(tag, "inertia") == 0) {
              // parse <inertia ixx="..." ixy="..." ixz="..." iyy="..."
              // iyz="..." izz="...">
              double ixx, ixy, ixz, iyy, iyz, izz;
              const char *s_ixx = elt->Attribute("ixx");
              const char *s_ixy = elt->Attribute("ixy");
              const char *s_ixz = elt->Attribute("ixz");
              const char *s_iyy = elt->Attribute("iyy");
              const char *s_iyz = elt->Attribute("iyz");
              const char *s_izz = elt->Attribute("izz");
              if (!s_ixx || !s_ixy || !s_ixz || !s_iyy || !s_iyz || !s_izz ||
                  std::sscanf(s_ixx, "%lf", &ixx) != 1 ||
                  std::sscanf(s_ixy, "%lf", &ixy) != 1 ||
                  std::sscanf(s_ixz, "%lf", &ixz) != 1 ||
                  std::sscanf(s_iyy, "%lf", &iyy) != 1 ||
                  std::sscanf(s_iyz, "%lf", &iyz) != 1 ||
                  std::sscanf(s_izz, "%lf", &izz) != 1) {
                throw std::runtime_error(
                    "Invalid inertia attributes in URDF for link " + link_name);
              }
              model.link_I[link_index] << ixx, ixy, ixz, ixy, iyy, iyz, ixz,
                  iyz, izz;
            }
          }
        }
      }

    } else if (strcmp(child->Value(), "joint") == 0) {
      // Parsing name
      std::string joint_name(child->Attribute("name"));
      if (!joint_name.size()) {
        throw std::runtime_error("Joint name attribute is missing");
      }
      // Getting index of the joint
      uint16_t joint_index = utils::jnt_name2id(model, joint_name);

      // Parsing type
      const char *type = child->Attribute("type");
      if (!type) {
        throw std::runtime_error("Joint type attribute is missing");
      }
      structs::JointType jointType = structs::getJointType(std::string(type));
      model.jnt_type[joint_index] = jointType;

      // Processing the joint dimension
      uint16_t jnt_qdof = structs::getJointQdof(jointType);
      if (jnt_qdof != 0) {
        model.jnt_qaddr[joint_index] = jnt_qdof_index;
        for (uint16_t i = 0; i < jnt_qdof; ++i) {
          model.qpos_jnt_id[jnt_qdof_index] = joint_index;
          ++jnt_qdof_index;
        }
      }
      uint16_t jnt_dof = structs::getJointDof(jointType);
      if (jnt_dof != 0) {
        model.jnt_dofadr[joint_index] = jnt_dof_index;
        for (uint16_t i = 0; i < jnt_dof; ++i) {
          model.dof_jnt_id[jnt_dof_index] = joint_index;
          ++jnt_dof_index;
        }
      }

      for (raisim::TiXmlElement *jnt_child = child->FirstChildElement();
           jnt_child != nullptr; jnt_child = jnt_child->NextSiblingElement()) {
        if (strcmp(jnt_child->Value(), "origin") == 0) {
          char const *xyz = jnt_child->Attribute("xyz");

          model.jnt_rel_pos[joint_index] =
              Eigen::Vector3d::Zero(); // Initialize to zero
          if (xyz) {
            if (std::sscanf(xyz, "%lf %lf %lf",
                            &model.jnt_rel_pos[joint_index][0],
                            &model.jnt_rel_pos[joint_index][1],
                            &model.jnt_rel_pos[joint_index][2]) != 3) {
              throw std::runtime_error(
                  "Invalid xyz attribute in URDF for joint " + joint_name);
            }
          } else {
            throw std::runtime_error(
                "Origin xyz attribute is missing in URDF for joint " +
                joint_name);
          }

          char const *rpy = jnt_child->Attribute("rpy");

          Eigen::Vector3d erpy = Eigen::Vector3d::Zero();
          if (rpy) {
            if (std::sscanf(rpy, "%lf %lf %lf", &erpy[0], &erpy[1], &erpy[2]) !=
                3) {
              throw std::runtime_error(
                  "Invalid rpy attribute in URDF for joint " + joint_name);
            }
          }
          model.jnt_rel_rot[joint_index] = spatial::rpy2rot(erpy);
        }
        if (strcmp(jnt_child->Value(), "limit") == 0) {
          const char *lower = jnt_child->Attribute("lower");
          const char *upper = jnt_child->Attribute("upper");

          model.jnt_range[joint_index] =
              Eigen::Vector2d::Zero(); // Initialize to zero
          if (lower) {
            if (std::sscanf(lower, "%lf", &model.jnt_range[joint_index][0]) !=
                1) {
              throw std::runtime_error(
                  "Invalid limit attribute in URDF for joint " + joint_name);
            }
          }
          if (upper) {
            if (std::sscanf(lower, "%lf", &model.jnt_range[joint_index][1]) !=
                1) {
              throw std::runtime_error(
                  "Invalid limit attribute in URDF for joint " + joint_name);
            }
          }
        }
        if (strcmp(jnt_child->Value(), "axis") == 0) {
          const char *axis_str = jnt_child->Attribute("xyz");
          // default axis is X
          Eigen::Vector3d axis3d = Eigen::Vector3d::UnitX();
          if (axis_str) {
            if (std::sscanf(axis_str, "%lf %lf %lf", &axis3d[0], &axis3d[1],
                            &axis3d[2]) != 3) {
              throw std::runtime_error(
                  "Invalid axis attribute in URDF for joint " + joint_name);
            }
          }

          // grab the 6D axis slot
          auto &ax6 = model.jnt_axis_local[joint_index];
          ax6.setZero();

          switch (model.jnt_type[joint_index]) {
          case structs::PRISMATIC:
            // translation only -> first 3 entries
            ax6.head<3>() = axis3d;
            break;

          case structs::REVOLUTE:
            // rotation only -> last 3 entries
            ax6.tail<3>() = axis3d;
            break;

          case structs::BALL:
            // ball joint -> fill all ones
            ax6.tail<3>().setOnes();
            break;

          case structs::FREE:
            // floating‐base or ball joint -> fill all ones
            ax6.setOnes();
            break;

          default:
            // FIXED or unsupported -> leave zero
            break;
          }
        }
      }
    }
  }

  return model;
}

inline structs::Model parseURDF(const std::string &urdf) {
  // Parse the URDF string
  return parseURDF(urdf, false);
}

inline structs::Model parseURDFfromFile(const std::string &urdf_path,
                                        const bool &floating_base) {
  // Read the URDF file from the given path
  std::ifstream urdf_file(urdf_path);
  std::string str;
  std::string file_contents;
  while (std::getline(urdf_file, str)) {
    file_contents += str;
    file_contents.push_back('\n');
  }

  return parseURDF(file_contents, floating_base);
}

inline structs::Model parseURDFfromFile(const std::string &urdf_path) {
  return parseURDFfromFile(urdf_path, false);
}
} // namespace parse
namespace algorithms {

namespace kinematics {
inline Eigen::MatrixXd
computeLinearJacobian(const dyn::structs::Model &model,
                      const dyn::structs::Data &data, const uint16_t &obj_id,
                      //  TODO: this should be implemented differently...
                      const bool &is_jnt, const Eigen::Vector3d &point) {

  Eigen::MatrixXd Jlin(3, model.nv);
  Jlin.setZero();
  Eigen::Vector3d r_ee;
  uint16_t jnt_id;
  if (is_jnt) {
    jnt_id = obj_id;
    r_ee = data.jnt_pos[jnt_id] + point;
  } else {
    jnt_id = model.link_parentid[obj_id];
    r_ee = data.link_i_pos[obj_id] + point;
  }
  while (jnt_id != UINT16_MAX) {
    Eigen::Vector3d r_jnt_ee = r_ee - data.jnt_pos[jnt_id];
    uint16_t dof_idx = model.jnt_dofadr[jnt_id];

    // TODO: does not support floating base
    auto jnt_type = model.jnt_type[jnt_id];
    if (jnt_type == structs::REVOLUTE || jnt_type == structs::PRISMATIC)
      Jlin.col(dof_idx) += data.jnt_axis[jnt_id].head<3>() +
                           data.jnt_axis[jnt_id].tail<3>().cross(r_jnt_ee);
    else if (jnt_type == structs::FREE) {
      Jlin(Eigen::all, Eigen::seqN(dof_idx, 3)) += Eigen::Matrix3d::Identity();
      Jlin(Eigen::all, Eigen::seqN(dof_idx + 3, 3)) +=
          -spatial::skew_matrix(r_jnt_ee);
    }
    jnt_id = model.link_parentid[model.jnt_parentid[jnt_id]];
  }

  return Jlin;
}
inline Eigen::MatrixXd computeAngularJacobian(const dyn::structs::Model &model,
                                              const dyn::structs::Data &data,
                                              const uint16_t &obj_id,
                                              const bool &is_jnt) {
  Eigen::MatrixXd Jang(3, model.nv);
  Jang.setZero();
  uint16_t jnt_id;
  if (is_jnt) {
    jnt_id = obj_id;
  } else {
    jnt_id = model.link_parentid[obj_id];
  }
  while (jnt_id != UINT16_MAX) {
    uint16_t dof_idx = model.jnt_dofadr[jnt_id];

    // TODO: does not support floating base
    auto jnt_type = model.jnt_type[jnt_id];
    if (jnt_type == structs::REVOLUTE || jnt_type == structs::PRISMATIC)
      Jang.col(dof_idx) += data.jnt_axis[jnt_id].tail(3);
    else if (jnt_type == structs::FREE) {
      Jang(Eigen::all, Eigen::seqN(dof_idx + 3, 3)) +=
          Eigen::Matrix3d::Identity();
    }
    jnt_id = model.link_parentid[model.jnt_parentid[jnt_id]];
  }

  return Jang;
}

inline Eigen::MatrixXd computeLinearJacobian(const dyn::structs::Model &model,
                                             const dyn::structs::Data &data,
                                             const uint16_t &jnt_id) {
  return computeLinearJacobian(model, data, jnt_id, true,
                               Eigen::Vector3d::Zero());
}

inline Eigen::MatrixXd computeAngularJacobian(const dyn::structs::Model &model,
                                              const dyn::structs::Data &data,
                                              const uint16_t &jnt_id) {
  return computeAngularJacobian(model, data, jnt_id, true);
}

inline Eigen::MatrixXd computeJacobian(const dyn::structs::Model &model,
                                       dyn::structs::Data &data,
                                       const uint16_t &obj_id,
                                       const bool &is_jnt,
                                       const Eigen::Vector3d &point) {
  Eigen::MatrixXd J(6, model.nv);
  J.block(0, 0, 3, model.nv) =
      computeLinearJacobian(model, data, obj_id, is_jnt, point);
  J.block(3, 0, 3, model.nv) =
      computeAngularJacobian(model, data, obj_id, is_jnt);
  return J;
}

inline void computeJandVel(const dyn::structs::Model &model,
                           dyn::structs::Data &data) {
  for (uint16_t i = 0; i < model.nl; ++i) {
    data.link_Jpos[i] =
        computeLinearJacobian(model, data, i, false, Eigen::Vector3d::Zero());
    data.link_lvel[i] = data.link_Jpos[i] * data.v;

    data.link_Jrot[i] = computeAngularJacobian(model, data, i, false);
    data.link_avel[i] = data.link_Jrot[i] * data.v;
  }

  for (uint16_t i = 0; i < model.nj; ++i) {
    data.jnt_Jpos[i] = computeLinearJacobian(model, data, i);
    data.jnt_lvel[i] = data.jnt_Jpos[i] * data.v;

    data.jnt_Jrot[i] = computeAngularJacobian(model, data, i);
    data.jnt_avel[i] = data.jnt_Jrot[i] * data.v;
  }
}

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
  data.M.setZero();
  Eigen::Matrix<double, 6, 6> I_c;
  Eigen::Vector<double, 6> F;

  for (int16_t i = model.nl - 1; i >= 1; --i) {
    uint16_t jnt_id = model.link_parentid[i];
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
      } else {
        data.M(dof_adr, other_dof) = F.dot(data.jnt_axis[jnt_id]);
        data.M(other_dof, dof_adr) = data.M(dof_adr, other_dof);
      }

      p_link_id = p_link_id_next;
      p_link_id_next = model.jnt_parentid[jnt_id];
    }
  }
}

} // namespace kinematics
inline void update(const dyn::structs::Model &model, dyn::structs::Data &data) {
  // Update the kinematics of the model based on the current state
  // This is a placeholder implementation
  kinematics::computeForwardKinematics(model, data);
  kinematics::computeCompositeMassInertia(model, data);
  // kinematics::computeJandVel(model, data);
  kinematics::computeMassMatrix(model, data);
}
} // namespace algorithms
  //
} // namespace dyn
  //
/// do not change the name of the method
inline Eigen::MatrixXd getMassMatrix(const Eigen::VectorXd &gc) {

  /// !!!!!!!!!! NO RAISIM FUNCTIONS HERE !!!!!!!!!!!!!!!!!
  std::filesystem::path urdfPath =
      std::filesystem::current_path().parent_path() / "resource" /
      "mini_cheetah" / "urdf" / "cheetah.urdf";
  dyn::structs::Model model =
      dyn::parse::parseURDFfromFile(urdfPath.string(), true);
  dyn::structs::Data data = dyn::structs::makeData(model);
  data.q = gc;
  dyn::algorithms::update(model, data);

  return data.M;
}