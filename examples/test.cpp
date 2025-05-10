#include "raisim/RaisimServer.hpp"
#include <cassert>
#include <chrono>
#include <dyn/algorithms/udpate.hpp>
#include <dyn/parse.hpp>
#include <dyn/structs.hpp>
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

// Test 1: forward kinematics
static void testKinematics(ModelHandles &h) {
  for (uint16_t jnt_id = 0; jnt_id < h.dmodel.nj; ++jnt_id) {
    std::string jnt_name = h.dmodel.jnt_name[jnt_id];

    Eigen::Vector3d pd = h.ddata.jnt_pos[jnt_id];
    raisim::Vec<3> pr;
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
  for (uint16_t jnt_id = 0; jnt_id < h.dmodel.nj; ++jnt_id) {
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

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <panda|minicheetah>\n";
    return -1;
  }
  auto name = std::string(argv[1]);
  try {
    auto h = loadModel(name);
    for (int i = 0; i < 1000000; ++i) {
      auto q = randConfig(
          h.rsys, std::chrono::system_clock::now().time_since_epoch().count());
      auto v = randVelocity(
          h.rsys, std::chrono::system_clock::now().time_since_epoch().count());
      h.rsys->setState(q, v);
      h.server->focusOn(h.rsys);
      h.server->launchServer();
      h.rsys->updateKinematics();
      h.world->integrate1();
      auto M = h.rsys->getMassMatrix();

      h.ddata.q = q;
      h.ddata.v = v;
      dyn::algorithms::update(h.dmodel, h.ddata);
      std::cout << "q: " << q.transpose() << "\n";
      std::cout << "v: " << v.transpose() << "\n";
      testKinematics(h);
      testVelocity(h);
      testLinkKinematics(h);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return -1;
  }
  return 0;
}