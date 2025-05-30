#ifndef DYN_PARSE_HPP
#define DYN_PARSE_HPP

#include "spatial.hpp"
#include "structs.hpp"
#include "utils.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stack>
#include <stdexcept>
#include <string>
#include <tinyxml_rai/tinystr.h>
#include <tinyxml_rai/tinyxml_rai.h>
#include <vector>

namespace dyn {
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
  model.jnt_dofadr.assign(model.nj, 0.0);

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
      
      model.jnt_rel_pos[joint_index] =
          Eigen::Vector3d::Zero(); // Initialize to zero
      model.jnt_rel_rot[joint_index] = Eigen::Matrix3d::Identity();
      model.jnt_axis_local[joint_index] = Eigen::Vector<double, 6>::Zero();
      model.jnt_range[joint_index] = Eigen::Vector2d::Zero();
      for (raisim::TiXmlElement *jnt_child = child->FirstChildElement();
           jnt_child != nullptr; jnt_child = jnt_child->NextSiblingElement()) {
        if (strcmp(jnt_child->Value(), "origin") == 0) {
          char const *xyz = jnt_child->Attribute("xyz");

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

          if (lower) {
            if (std::sscanf(lower, "%lf", &model.jnt_range[joint_index][0]) !=
                1) {
              throw std::runtime_error(
                  "Invalid limit attribute in URDF for joint " + joint_name);
            }
          }
          if (upper) {
            if (std::sscanf(upper, "%lf", &model.jnt_range[joint_index][1]) !=
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

          switch (model.jnt_type[joint_index]) {
          case structs::PRISMATIC:
            // translation only -> first 3 entries
            model.jnt_axis_local[joint_index].head<3>() = axis3d;
            break;

          case structs::REVOLUTE:
            // rotation only -> last 3 entries
            model.jnt_axis_local[joint_index].tail<3>() = axis3d;
            break;

          case structs::BALL:
            // ball joint -> fill all ones
            model.jnt_axis_local[joint_index].tail<3>().setOnes();
            break;

          case structs::FREE:
            // floating‐base or ball joint -> fill all ones
            model.jnt_axis_local[joint_index].setOnes();
            break;

          case structs::FIXED:
            // FIXED-> leave zero
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

} // namespace dyn
#endif // DYN_PARSE_HPP