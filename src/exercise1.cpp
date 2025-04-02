//
// Created by Jemin Hwangbo on 2022/03/17.
//

#include <chrono>
#include <climits>
#define _MAKE_STR(x) __MAKE_STR(x)
#define __MAKE_STR(x) #x

#include "exercise1_20254024.hpp"
#include "raisim/RaisimServer.hpp"

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

int main(int argc, char *argv[]) {
  // create raisim world
  raisim::World world;                 // physics world
  raisim::RaisimServer server(&world); // visualization server
  world.addGround();

  // panda
  auto panda = world.addArticulatedSystem(std::string(_MAKE_STR(RESOURCE_DIR)) +
                                          "/Panda/panda.urdf");
  panda->setName("panda");
  server.focusOn(panda);

  // panda configuration
  Eigen::VectorXd jointNominalConfig(panda->getGeneralizedCoordinateDim());
  {
    // Define lower and upper joint limits for each of the 9 joints.
    // (Adjust the limits as needed for your specific robot.)
    auto limits = panda->getJointLimits();

    // Setup random generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Generate random values for each joint within its limits
    for (int i = 0; i < jointNominalConfig.size(); ++i) {
      std::uniform_real_distribution<double> dis(limits[0], limits[1]);
      jointNominalConfig[i] = dis(gen);
    }
  }

  server.launchServer();
  try {
    auto debugSphere = server.addVisualSphere("debug_sphere", 0.02);
    auto answerSphere = server.addVisualSphere("answer_sphere", 0.02);
    answerSphere->setColor(0, 1, 0, 1);
    debugSphere->setColor(1, 0, 0, 1);
    for (int i = 0; i < 10000000; i++) {
      jointNominalConfig = generateRandomJointConfig(
          panda,
          static_cast<unsigned int>(
              std::chrono::system_clock::now().time_since_epoch().count()));
      panda->setGeneralizedCoordinate(jointNominalConfig);
      panda->updateKinematics();

      // debug sphere
      Eigen::Vector3d computed_pos = getEndEffectorPosition(jointNominalConfig);
      debugSphere->setPosition(computed_pos);

      // solution sphere
      raisim::Vec<3> pos;
      panda->getFramePosition("panda_finger_joint3", pos);
      answerSphere->setPosition(pos.e());

      std::cout << "End effector position: " << computed_pos << std::endl;
      std::cout << "Solution: " << pos << std::endl;

      assert((computed_pos - pos.e()).norm() < 1e-6);

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
