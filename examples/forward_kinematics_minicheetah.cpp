#include "dyn.hpp"
#include "raisim/RaisimServer.hpp"
#include <chrono>

inline Eigen::Vector3d getEndEffectorPosition(dyn::structs::Model &model,
                                              dyn::structs::Data &data,
                                              const Eigen::VectorXd &gc) {
  // Updating the model
  data.q = gc;
  data.v = Eigen::VectorXd::Zero(model.nv); // Assuming zero velocity
  dyn::algorithms::update(model, data);

  // Get the end effector position
  return data
      .jnt_pos[dyn::utils::jnt_name2id(model, "RB_JOINT3")]; /// replace this
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
  // Get default joint limits
  auto limits = robot->getJointLimits();

  // --------------------------------------------------
  // Override for Minicheetah:
  // Torso position: X, Y in [-0.5, 0.5], height Z in [0.2, 0.5]
  limits[0] = {-0.5, 0.5}; // base X
  limits[1] = {-0.5, 0.5}; // base Y
  limits[2] = {0.2, 0.5};  // base Z
  limits[3] = {1.0, 1.0};  // base roll
  limits[4] = {0.0, 0.0};  // base pitch
  limits[5] = {0.0, 0.0};  // base yaw
  limits[6] = {0.0, 0.0};  // base quaternion (w, x, y, z)

  // (quaternion at indices 3–6 will be overwritten later with a fixed
  // orientation)

  // Legs: each of the 12 joint dims (3 joints × 4 legs) => indices 7..18
  for (int i = 7; i < limits.size(); ++i) {
    limits[i] = {-0.5, 0.5}; // joint limits for legs
  }

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
  jointConfig[3] = 1.0;
  jointConfig[4] = 0.0;
  jointConfig[5] = 0.0;
  jointConfig[6] = 0.0;

  std::cout << "Joint config: " << jointConfig.transpose() << std::endl;

  return jointConfig;
}

int main(int argc, char *argv[]) {
  // build full URDF path
  std::filesystem::path urdfPath =
      std::filesystem::current_path().parent_path() / "resource" /
      "mini_cheetah" / "urdf" / "cheetah.urdf";

  // create raisim world
  raisim::World world;
  raisim::RaisimServer server(&world);
  world.addGround();

  // load panda
  auto raisim_robot = world.addArticulatedSystem(urdfPath.string());
  raisim_robot->setName("panda");
  server.focusOn(raisim_robot);

  //   Load our own model
  dyn::structs::Model model =
      dyn::parse::parseURDFfromFile(urdfPath.string(), true);
  dyn::utils::printModelInfo(model);
  dyn::structs::Data data = dyn::structs::makeData(model);

  // Check that the model is valid
  // dyn::utils::printModelInfo(model);

  // panda configuration
  Eigen::VectorXd jointNominalConfig(
      raisim_robot->getGeneralizedCoordinateDim());
  {
    auto limits = raisim_robot->getJointLimits();
    std::random_device rd;
    std::mt19937 gen(rd());
    for (int i = 0; i < jointNominalConfig.size(); ++i) {
      std::uniform_real_distribution<double> dis(limits[i][0], limits[i][1]);
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
          raisim_robot,
          static_cast<unsigned int>(
              std::chrono::system_clock::now().time_since_epoch().count()));
      raisim_robot->setGeneralizedCoordinate(jointNominalConfig);
      raisim_robot->updateKinematics();

      // debug sphere
      Eigen::Vector3d computed_pos =
          getEndEffectorPosition(model, data, jointNominalConfig);
      debugSphere->setPosition(computed_pos);

      // solution sphere
      raisim::Vec<3> pos;
      raisim_robot->getFramePosition("panda_finger_joint3", pos);
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
