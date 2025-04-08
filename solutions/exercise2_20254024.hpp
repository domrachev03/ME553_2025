#ifndef ME553_2022_SOLUTIONS_EXERCISE2_20254024_HPP_
#define ME553_2022_SOLUTIONS_EXERCISE2_20254024_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <arm_neon.h>
#include <dyn.hpp>
#include <sys/types.h>
#include <tinyxml_rai/tinystr.h>
#include <tinyxml_rai/tinyxml_rai.h>

/// do not change the name of the method
inline Eigen::Vector3d getLinearVelocity(const Eigen::VectorXd &gc,
                                         const Eigen::VectorXd &gv) {
  //////////////////////////
  ///// Your Code Here /////
  //////////////////////////
  std::filesystem::path urdfPath =
      std::filesystem::current_path().parent_path() / "resource" / "Panda" /
      "panda.urdf";
  dyn::structs::Model model = dyn::parse::parseURDFfromFile(urdfPath.string());
  dyn::structs::Data data = dyn::structs::makeData(model);

  // Check that the model is valid
  // dyn::utils::printModelInfo(model);

  // Updating the model
  data.q = gc;
  data.v = gv;
  dyn::algorithms::update(model, data);

  Eigen::MatrixXd Jlin = dyn::algorithms::kinematics::computeLinearJacobian(
      model, data, dyn::utils::jnt_name2id(model, "panda_finger_joint3"), true,
      Eigen::Vector3d::Zero());
  Eigen::Vector3d lin_vel = Jlin * gv;

  return lin_vel; /// replace this
}

/// do not change the name of the method
inline Eigen::Vector3d getAngularVelocity(const Eigen::VectorXd &gc,
                                          const Eigen::VectorXd &gv) {
  //////////////////////////
  ///// Your Code Here /////
  //////////////////////////
  std::filesystem::path urdfPath =
      std::filesystem::current_path().parent_path() / "resource" / "Panda" /
      "panda.urdf";
  dyn::structs::Model model = dyn::parse::parseURDFfromFile(urdfPath.string());
  dyn::structs::Data data = dyn::structs::makeData(model);

  // Check that the model is valid
  // dyn::utils::printModelInfo(model);

  // Updating the model
  data.q = gc;
  data.v = gv;
  dyn::algorithms::update(model, data);
  Eigen::MatrixXd Jrot = dyn::algorithms::kinematics::computeAngularJacobian(
      model, data, dyn::utils::jnt_name2id(model, "panda_finger_joint3"), true);
  Eigen::Vector3d ang_vel = Jrot * gv;

  return ang_vel; /// replace this
}
#endif
