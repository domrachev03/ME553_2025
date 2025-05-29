#ifndef DYN_ALGORITHMS_UPDATE_HPP
#define DYN_ALGORITHMS_UPDATE_HPP

#include "../structs.hpp"
#include "acceleration.hpp"
#include "dynamics.hpp"
#include "kinematics.hpp"
#include "velocity.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dyn {
namespace algorithms {
inline void update(const dyn::structs::Model &model, dyn::structs::Data &data) {
  // Update the kinematics of the model based on the current state
  // This is a placeholder implementation
  kinematics::computeForwardKinematics(model, data);
  kinematics::computeCompositeMassInertia(model, data);
  kinematics::computeMassMatrix(model, data);

  velocity::computeVelocity(model, data);

  dynamics::computeBias(model, data);
}
} // namespace algorithms
} // namespace dyn
#endif // DYN_ALGORITHMS_UPDATE_HPP