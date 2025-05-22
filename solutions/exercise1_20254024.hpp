#ifndef ME553_2022_SOLUTIONS_EXERCISE1_20254024_HPP_
#define ME553_2022_SOLUTIONS_EXERCISE1_20254024_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dyn.hpp>
#include <filesystem>
#include <sys/types.h>
#include <tinyxml_rai/tinystr.h>
#include <tinyxml_rai/tinyxml_rai.h>

/// do not change the name of the method
inline Eigen::Vector3d getEndEffectorPosition(const Eigen::VectorXd &gc) {
  std::filesystem::path urdfPath =
      std::filesystem::current_path().parent_path() / "resource" / "Panda" /
      "panda.urdf";
  dyn::structs::Model model = dyn::parse::parseURDFfromFile(urdfPath.string());
  dyn::structs::Data data = dyn::structs::makeData(model);

  // Check that the model is valid
  // dyn::utils::printModelInfo(model);

  // Updating the model
  data.q = gc;
  data.v = Eigen::VectorXd::Zero(model.nv); // Assuming zero velocity
  dyn::algorithms::update(model, data);

  // Get the end effector position
  return data.jnt_pos[dyn::utils::jnt_name2id(
      model, "panda_finger_joint3")]; /// replace this
}

#endif // ME553_2022_SOLUTIONS_EXERCISE1_20254024_HPP_