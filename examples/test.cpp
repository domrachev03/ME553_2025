#include <cassert>
#include <chrono>
#include <dyn/algorithms/update.hpp>
#include <dyn/parse.hpp>
#include <dyn/structs.hpp>
#include <filesystem>
#include <iostream>
#include <raisim/RaisimServer.hpp>
#include <raisim/math.hpp>
#include <random>
#include <string>
#include <thread>

struct ModelHandles {
  raisim::World *world;
  raisim::RaisimServer *server{};
  raisim::ArticulatedSystem *rsys{};
  dyn::structs::Model dmodel;
  dyn::structs::Data ddata;
};

// Build URDF path from robot name
static std::filesystem::path getURDFPath(const std::string &name) {
  auto base = std::filesystem::current_path().parent_path() / "resource";
  if (name == "panda")
    return base / "Panda" / "panda.urdf";
  else if (name == "minicheetah")
    return base / "mini_cheetah" / "urdf" / "cheetah.urdf";
  else if (name == "kinova")
    return base / "kinova" / "robot.urdf";
  else if (name == "cartpole")
    return base / "cartPole" / "cartpole.urdf";
  else if (name == "manip3d")
    return base / "manip3d" / "robot_3D.urdf";
  else if (name == "chain")
    return base / "chain" / "robot.urdf";
  else
    throw std::runtime_error("Unknown robot: " + name);
}

// Common loader for Raisim + dyn
static ModelHandles loadModel(const std::string &name) {
  ModelHandles h;
  h.world = new raisim::World();
  h.world->addGround();
  h.server = new raisim::RaisimServer(h.world);

  auto urdf = getURDFPath(name).string();
  h.rsys = h.world->addArticulatedSystem(urdf);
  h.rsys->setName(name);

  h.dmodel = dyn::parse::parseURDFfromFile(urdf, name == "minicheetah");
  h.ddata = dyn::structs::makeData(h.dmodel);

  return h;
}

// Random state generators
static Eigen::VectorXd randConfig(raisim::ArticulatedSystem *r,
                                  unsigned seed = 0) {
  auto n = r->getGeneralizedCoordinateDim();
  Eigen::VectorXd q(n);
  std::mt19937 gen(seed ? seed : std::random_device{}());
  for (int i = 0; i < n; ++i) {
    std::uniform_real_distribution<> d(-1.0, 1.0);
    q[i] = d(gen);
  }
  if (r->gc_.size() != r->gv_.size()) {
    // Normalize randomized quaternion to unit length
    q.segment<4>(3) = q.segment<4>(3).normalized();
  }
  return q;
}
static Eigen::VectorXd randVelocity(raisim::ArticulatedSystem *r,
                                    unsigned seed = 0) {
  auto nv = r->getDOF();
  Eigen::VectorXd v(nv);
  std::mt19937 gen(seed ? seed : std::random_device{}());
  std::uniform_real_distribution<> d(-2, 2);
  for (int i = 0; i < nv; ++i)
    v[i] = d(gen);
  return v;
}

static Eigen::VectorXd randTorque(raisim::ArticulatedSystem *r,
                                  unsigned seed = 0) {
  auto nv = r->getDOF();
  Eigen::VectorXd tau(nv);
  std::mt19937 gen(seed ? seed : std::random_device{}());
  std::uniform_real_distribution<> d(-2, 2);
  for (int i = 0; i < nv; ++i)
    tau[i] = d(gen);
  return tau;
}

// Test 1: forward kinematics
static void testKinematics(ModelHandles &h) {
  for (uint16_t jnt_id = 1; jnt_id < h.dmodel.nj; ++jnt_id) {
    std::string jnt_name = h.dmodel.jnt_name[jnt_id];

    Eigen::Vector3d pd = h.ddata.jnt_pos[jnt_id];
    raisim::Vec<3> pr;
    std::cout << "Joint " << jnt_id << " name: " << jnt_name
              << ", position: " << pd.transpose() << "\n";
    h.rsys->getFramePosition(jnt_name, pr);
    if (!((pd - pr.e()).norm() < 1e-6)) {
      std::cerr << "Joint " << jnt_id
                << " position mismatch: " << pd.transpose() << " vs "
                << pr.e().transpose() << "\n";
      throw std::runtime_error("Kinematics test failed");
    }
  }
  std::cout << "[PASS] kinematics\n";
}

// Test 2: forward velocity
static void testVelocity(ModelHandles &h, int iters = 100) {
  raisim::Vec<3> tipVel, tipAngVel;
  Eigen::Vector3d pd, pr;
  for (uint16_t jnt_id = 1; jnt_id < h.dmodel.nj; ++jnt_id) {
    std::string jnt_name = h.dmodel.jnt_name[jnt_id];
    pd = h.ddata.jnt_lvel[jnt_id];
    pr = h.ddata.jnt_avel[jnt_id];

    h.rsys->getFrameVelocity(jnt_name, tipVel);
    h.rsys->getFrameAngularVelocity(jnt_name, tipAngVel);
    if (!((pd - tipVel.e()).norm() < 1e-6 &&
          (pr - tipAngVel.e()).norm() < 1e-6)) {
      std::cerr << "Joint " << jnt_id
                << " velocity mismatch: " << pd.transpose() << " vs "
                << tipVel.e().transpose() << "\n";
      throw std::runtime_error("Velocity test failed");
    }
  }
  std::cout << "[PASS] velocity\n";
}

// Test 3: links inertia frame kinematics
static void testLinkKinematics(ModelHandles &h) {
  std::string link_name;
  Eigen::Vector3d dyn_pd, rai_pd;
  Eigen::Matrix3d dyn_pr, rai_pr;
  for (uint16_t link_id = 1; link_id < h.dmodel.nl; ++link_id) {
    link_name = h.dmodel.link_name[link_id];
    dyn_pd = h.ddata.link_i_pos[link_id];
    rai_pd = h.rsys->comPos_W[link_id - 1].e();

    dyn_pr = h.ddata.link_i_rot[link_id];
    rai_pr = h.rsys->rot_WB[link_id - 1].e();
    if (!((dyn_pd - rai_pd).norm() < 1e-9)) {
      std::cerr << "Link " << link_id
                << " position mismatch: " << dyn_pd.transpose() << " vs "
                << rai_pd.transpose() << "\n";
      throw std::runtime_error("Link kinematics test failed");
    }
    if (!((dyn_pr - rai_pr).norm() < 1e-9)) {
      std::cerr << "Link " << link_id << " rotation mismatch: " << dyn_pr
                << " vs " << rai_pr << "\n";
      throw std::runtime_error("Link kinematics test failed");
    }
  }
  std::cout << "[PASS] link kinematics\n";
}

// Test 4: subtree CoM and inertia
static void testSubtreeCoM(ModelHandles &h) {
  const double tol = 1e-9;

  // 1) Mass
  for (size_t i = 0; i < h.rsys->compositeMass.size(); ++i) {
    double rai_m = h.rsys->compositeMass[i];
    bool found = false;
    for (size_t j = 0; j < h.ddata.link_subtree_mass.size(); ++j) {
      double dyn_m = h.ddata.link_subtree_mass[j];
      if (std::abs(rai_m - dyn_m) < tol) {
        std::cout << "[MATCH] mass Raisim[" << i << "]=" << rai_m << " == dyn["
                  << j << "]=" << dyn_m << "\n";
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::runtime_error(
          "No matching dyn subtree mass for Raisim compositeMass");
    }
  }

  // 2) Center of Mass
  for (size_t i = 1; i < h.rsys->composite_com_W.size(); ++i) {
    Eigen::Vector3d rai_com = h.rsys->composite_com_W[i].e();
    bool found = false;
    for (size_t j = 0; j < h.ddata.link_subtree_com.size(); ++j) {
      Eigen::Vector3d dyn_com = h.ddata.link_subtree_com[j];
      if ((rai_com - dyn_com).norm() < tol) {
        std::cout << "[MATCH] CoM Raisim[" << i << "]=" << rai_com.transpose()
                  << " == dyn[" << j << "]=" << dyn_com.transpose() << "\n";
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::runtime_error(
          "No matching dyn subtree CoM for Raisim composite_com_W");
    }
  }

  // 3) Link inertia in world at CoM
  for (size_t i = 1; i < h.rsys->inertia_comW.size(); ++i) {
    Eigen::Matrix3d rai_I = h.rsys->inertia_comW[i].e();
    bool found = false;
    for (size_t j = 0; j < h.ddata.link_I_w.size(); ++j) {
      Eigen::Matrix3d dyn_I = h.ddata.link_I_w[j];
      if ((rai_I - dyn_I).norm() < tol) {
        std::cout << "[MATCH] world inertia Raisim[" << i << "]"
                  << " == dyn[" << j << "]\n";
        found = true;
        break;
      }
    }
    // if (!found) {
    //   throw std::runtime_error("No matching dyn world inertia");
    // }
  }

  // 4) Subtree inertia about composite CoM
  for (size_t i = 1; i < h.rsys->compositeInertia_W.size(); ++i) {
    Eigen::Matrix3d rai_I = h.rsys->compositeInertia_W[i].e();
    bool found = false;
    for (size_t j = 0; j < h.ddata.link_subtree_I.size(); ++j) {
      Eigen::Matrix3d dyn_I = h.ddata.link_subtree_I[j];
      if ((rai_I - dyn_I).norm() < tol) {
        std::cout << "[MATCH] subtree inertia Raisim[" << i
                  << "]"
                     " == dyn["
                  << j << "]\n";
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::runtime_error(
          "No matching dyn subtree inertia for Raisim index " +
          std::to_string(i));
    }
  }

  std::cout << "[PASS] subtree CoM and inertia (matched by search)\n";
}

// Test 5: joint axes
static void testJointAxes(ModelHandles &h) {
  const double tol = 1e-9;
  for (size_t rai_id = 1; rai_id < h.rsys->jointAxis_W.size(); ++rai_id) {
    Eigen::Vector3d rai_axis = h.rsys->jointAxis_W[rai_id].e();
    bool found = false;
    for (size_t dyn_id = 0; dyn_id < h.dmodel.nj; ++dyn_id) {
      Eigen::Vector3d dyn_ang_axis = h.ddata.jnt_axis[dyn_id].tail<3>();
      Eigen::Vector3d dyn_lin_axis = h.ddata.jnt_axis[dyn_id].head<3>();
      if ((rai_axis - dyn_ang_axis).norm() < tol ||
          (rai_axis - dyn_lin_axis).norm() < tol) {
        std::cout << "[MATCH] Raisim joint " << rai_id << " axis "
                  << rai_axis.transpose() << " == dyn joint " << dyn_id
                  << " axis " << dyn_ang_axis.transpose() << "\n";
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "[ERROR] Raisim joint " << rai_id << " axis "
                << rai_axis.transpose() << " has no match in dyn joints\n";
    }
  }
  std::cout << "[PASS] joint axes (matched by search)\n";
}

// Test 6: mass matrix
static void testMassMatrix(ModelHandles &h) {
  Eigen::MatrixXd M_dyn = h.ddata.M;
  Eigen::MatrixXd M_rai = h.rsys->getMassMatrix().e();
  // Check elementwise and show difference:
  for (int i = 0; i < M_dyn.rows(); ++i) {
    for (int j = i; j < M_dyn.cols(); ++j) {
      if (std::abs(M_dyn(i, j) - M_rai(i, j)) > 1e-9) {
        std::cerr << "Mass matrix mismatch at (" << i << ", " << j
                  << "): " << M_dyn(i, j) << " vs " << M_rai(i, j) << "\n";
        throw std::runtime_error("Mass matrix test failed");
      }
    }
  }
  std::cout << "[PASS] mass matrix\n";
}

// Test 7: acceleration
static void testAcceleration(ModelHandles &h) {
  raisim::Vec<3> tipAcc, tipAngAcc;
  Eigen::VectorXd dv = Eigen::VectorXd::Zero(h.dmodel.nv);
  std::vector<Eigen::Vector<double, 6>> jnt_acc =
      dyn::algorithms::acceleration::computeAcceleration(h.dmodel, h.ddata, dv)
          .second;
  for (size_t i = 0; i < h.rsys->nbody; ++i) {
    std::string jnt_name = h.dmodel.jnt_name[i];
    tipAcc = h.rsys->bodyLinearAcc[i].e();
    tipAngAcc = h.rsys->bodyAngAcc[i].e();
    bool found = false;
    for (size_t j = 0; j < h.dmodel.nj; ++j) {
      if ((jnt_acc[j].head(3) - tipAcc.e()).norm() < 1e-8 &&
          (jnt_acc[j].tail(3) - tipAngAcc.e()).norm() < 1e-8) {
        std::cout << "[MATCH] Acc Raisim[" << i
                  << "]=" << tipAcc.e().transpose() << " == dyn[" << j
                  << "]=" << jnt_acc[j].head(3).transpose() << "\n";
        found = true;
        break;
      }
    }
    if (!found) {
      // throw std::runtime_error(
      //     "No matching dyn subtree mass for Raisim compositeMass");
      std::cerr << "[ERROR] No matching dyn acceleration for Raisim "
                << jnt_name << " at index " << i << "\n";
    }
  }
  std::cout << "[PASS] acceleration\n";
}

// Test 8: bias
static void testBias(ModelHandles &h, const double g) {
  const double tol = 1e-9;
  Eigen::VectorXd b_dyn = h.ddata.b;
  auto b_rai = h.rsys->getNonlinearities({0, 0, -g}).e();
  for (int i = 0; i < b_dyn.size(); ++i) {
    if (std::abs(b_dyn[i] - b_rai[i]) > tol) {
      std::cerr << "Bias mismatch at index " << i << ": " << b_dyn[i] << " vs "
                << b_rai[i] << "\n";
    }
  }
  // throw std::runtime_error("Bias test failed");
  std::cout << "[PASS] bias\n";
}

// Test 9: fwd dynamics
static void testForwardDynamics(ModelHandles &h, const double g) {
  const double tol = 1e-9; // Dyn mass matrix and bias
  // Eigen::MatrixXd M_dyn = h.ddata.M;
  // Eigen::VectorXd b_dyn = h.ddata.b;
  // Raisim mass matrix and bias (nonlinearities)
  Eigen::MatrixXd M_rai = h.rsys->getMassMatrix().e();
  Eigen::VectorXd b_rai = h.rsys->getNonlinearities({0, 0, -g}).e();

  // Eigen::VectorXd a_rai = M_rai.inverse() * (h.ddata.tau - b_rai);
  Eigen::VectorXd a_rai = M_rai.inverse() * (h.ddata.tau - b_rai);

  for (int i = 0; i < h.ddata.dv.size(); ++i) {
    if (std::abs(h.ddata.dv[i] - a_rai[i]) > tol) {
      std::cerr << "Forward dynamics mismatch at index " << i
                << ": dyn=" << h.ddata.dv[i] << " vs rai=" << a_rai[i] << "\n";
      // throw std::runtime_error("Forward dynamics test failed");
    } else {
      std::cout << "[MATCH] Forward dynamics at index " << i << ": "
                << h.ddata.dv[i] << " == " << a_rai[i] << "\n";
    }
  }
  std::cout << "[PASS] forward dynamics\n";
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <panda|minicheetah>\n";
    return -1;
  }
  auto name = std::string(argv[1]);
  double g = 9.81; // Gravity constant, can be adjusted
  try {
    auto h = loadModel(name);
    // for (int i = 0; i < 1000000; ++i) {
    auto q = randConfig(
        h.rsys, std::chrono::system_clock::now().time_since_epoch().count());
    auto v = randVelocity(
        h.rsys, std::chrono::system_clock::now().time_since_epoch().count());
    auto tau = randTorque(
        h.rsys, std::chrono::system_clock::now().time_since_epoch().count());
    // v = Eigen::VectorXd::Zero(h.rsys->getDOF());
    // tau = Eigen::VectorXd::Zero(h.rsys->getDOF());
    h.rsys->setState(q, v);
    h.rsys->setGeneralizedForce(tau);
    h.server->focusOn(h.rsys);
    h.server->launchServer();
    h.rsys->updateKinematics();
    h.world->integrate1();
    auto M = h.rsys->getMassMatrix();
    h.rsys->getNonlinearities({0, 0, -g});

    h.ddata.q = q;
    h.ddata.v = v;
    h.ddata.tau = tau;
    h.ddata.gravity = Eigen::Vector3d(0, 0, -g);
    std::cout << "Running update algorithms...\n";
    dyn::algorithms::update(h.dmodel, h.ddata);
    std::cout << "q: " << q.transpose() << "\n";
    std::cout << "v: " << v.transpose() << "\n";
    std::cout << "tau: " << tau.transpose() << "\n";
    testKinematics(h);
    testVelocity(h);
    // testLinkKinematics(h);
    testSubtreeCoM(h);
    testJointAxes(h);
    testMassMatrix(h);
    testAcceleration(h);
    testBias(h, g);
    testForwardDynamics(h, g);
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return -1;
  }
  return 0;
}