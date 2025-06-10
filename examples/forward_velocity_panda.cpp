#include "raisim/RaisimServer.hpp"
#include <chrono>
#include <filesystem>
#include <dyn/algorithms/update.hpp>
#include <dyn/parse.hpp>
#include <dyn/structs.hpp>
#include <filesystem>

inline Eigen::Vector3d getLinearVelocity(dyn::structs::Model &model,
                                         dyn::structs::Data &data,
                                         const Eigen::VectorXd &gc,
                                         const Eigen::VectorXd &gv) {
  // Updating the model
  data.q = gc;
  data.v = gv;
  dyn::algorithms::update(model, data);

  return data.jnt_lvel[dyn::utils::jnt_name2id(model, "panda_finger_joint3")];
}

/// do not change the name of the method
inline Eigen::Vector3d getAngularVelocity(dyn::structs::Model &model,
                                          dyn::structs::Data &data,
                                          const Eigen::VectorXd &gc,
                                          const Eigen::VectorXd &gv) {
  // Updating the model
  data.q = gc;
  data.v = gv;
  dyn::algorithms::update(model, data);

  return data.jnt_avel[dyn::utils::jnt_name2id(model, "panda_finger_joint3")];
}
/**
 * Generates a random configuration for the robot joints
 * @param robot The articulated system (robot)
 * @param seed Random seed for reproducibility
 * @return Random joint configuration vector
 */
Eigen::VectorXd generateRandomJointConfig(raisim::ArticulatedSystem *robot,
                                          unsigned int seed = 0) {
  // Get the robot's configuration dimension
  Eigen::VectorXd jointConfig(robot->getGeneralizedCoordinateDim());

  // Get joint limits
  auto limits = robot->getJointLimits();
  limits[limits.size() - 2] = Eigen::Vector2d(-0.1, 0.1);
  limits[limits.size() - 1] = Eigen::Vector2d(-0.1, 0.1);

  // Setup random generator with provided seed
  // Use seed if provided, otherwise create a new random seed
  std::mt19937 gen;
  if (seed == 0) {
    std::random_device rd;
    gen.seed(rd());
  } else {
    gen.seed(seed);
  }

  // Generate random values for each joint within its limits
  for (int i = 0; i < jointConfig.size(); ++i) {
    std::uniform_real_distribution<double> dis(limits[i][0], limits[i][1]);
    jointConfig[i] = dis(gen);
  }

  return jointConfig;
}
Eigen::VectorXd generateRandomJointVelocity(raisim::ArticulatedSystem *robot,
                                            unsigned int seed = 0) {
  // Get the robot's configuration dimension
  Eigen::VectorXd jointVelocity(robot->getDOF());

  // Setup random generator with provided seed
  // Use seed if provided, otherwise create a new random seed
  std::mt19937 gen;
  if (seed == 0) {
    std::random_device rd;
    gen.seed(rd());
  } else {
    gen.seed(seed);
  }

  // Generate random values for each joint within its limits
  for (int i = 0; i < jointVelocity.size(); ++i) {
    std::uniform_real_distribution<double> dis(-2, 2);
    jointVelocity[i] = dis(gen);
  }

  return jointVelocity;
}

int main(int argc, char *argv[]) {
  // build full URDF path
  std::filesystem::path urdfPath =
      std::filesystem::current_path().parent_path() / "resource" / "Panda" /
      "panda.urdf";

  // create raisim world
  raisim::World world;
  raisim::RaisimServer server(&world);
  world.addGround();

  // load panda
  auto raisim_robot = world.addArticulatedSystem(urdfPath.string());
  raisim_robot->setName("panda");
  server.focusOn(raisim_robot);

  //   Load our own model
  dyn::structs::Model model = dyn::parse::parseURDFfromFile(urdfPath.string());
  dyn::structs::Data data = dyn::structs::makeData(model);

  // Check that the model is valid
  // dyn::utils::printModelInfo(model);

  // panda configuration
  Eigen::VectorXd jointNominalConfig(
      raisim_robot->getGeneralizedCoordinateDim());
  Eigen::VectorXd jointVelocity(raisim_robot->getDOF());
  raisim::Vec<3> tipVel, tipAngVel;

  server.launchServer();
  try {
    auto debugSphere = server.addVisualSphere("debug_sphere", 0.02);
    auto answerSphere = server.addVisualSphere("answer_sphere", 0.02);
    answerSphere->setColor(0, 1, 0, 1);
    debugSphere->setColor(1, 0, 0, 1);

    for (int i = 0; i < 10000000; i++) {
      jointNominalConfig = generateRandomJointConfig(
          raisim_robot,
          static_cast<unsigned int>(
              std::chrono::system_clock::now().time_since_epoch().count()));
      jointVelocity = generateRandomJointVelocity(
          raisim_robot,
          static_cast<unsigned int>(
              std::chrono::system_clock::now().time_since_epoch().count()));
      raisim_robot->setState(jointNominalConfig, jointVelocity);
      raisim_robot->getFrameVelocity("panda_finger_joint3", tipVel);
      raisim_robot->getFrameAngularVelocity("panda_finger_joint3", tipAngVel);

      // solution sphere

      if ((tipVel.e() -
           getLinearVelocity(model, data, jointNominalConfig, jointVelocity))
              .norm() < 1e-8) {
        std::cout << "the linear velocity is correct " << std::endl;
      } else {
        std::cout << "the linear velocity is not correct."
                  << getLinearVelocity(model, data, jointNominalConfig,
                                       jointVelocity)
                         .transpose()
                  << " vs " << tipVel.e().transpose() << std::endl;
      }

      if ((tipAngVel.e() -
           getAngularVelocity(model, data, jointNominalConfig, jointVelocity))
              .norm() < 1e-8) {
        std::cout << "the angular velocity is correct" << std::endl;
      } else {
        std::cout << "the angular velocity is not correct. It should be "
                  << tipAngVel.e().transpose() << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    server.killServer();
  } catch (...) {
    std::cerr << "Unknown exception caught." << std::endl;
    server.killServer();
  }
}
