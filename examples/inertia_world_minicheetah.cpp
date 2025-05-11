#include "raisim/RaisimServer.hpp"
#include <chrono>
#include <dyn/algorithms/update.hpp>
#include <dyn/parse.hpp>
#include <dyn/structs.hpp>

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
  raisim_robot->setName("minicheetah");
  server.focusOn(raisim_robot);

  //   Load our own model
  dyn::structs::Model model =
      dyn::parse::parseURDFfromFile(urdfPath.string(), true);
  dyn::structs::Data data = dyn::structs::makeData(model);

  // Check that the model is valid
  // dyn::utils::printModelInfo(model);

  Eigen::VectorXd jointNominalConfig(
      raisim_robot->getGeneralizedCoordinateDim());

  auto raisim_inertias = raisim_robot->getInertia();
  std::cout << "Length of inertia vector: " << raisim_inertias.size() << " vs "
            << model.nl << std::endl;
  // Print inertia for all links in the model
  for (uint16_t i = 1; i < model.nl; ++i) {
    std::cout << "Link " << i << ":" << std::endl;
    std::cout << "    Inertia in raisim: " << std::endl
              << raisim_inertias[i - 1];
    std::cout << "    Inertia in dyn: " << std::endl
              << model.link_I[i] << std::endl;
  }
}
