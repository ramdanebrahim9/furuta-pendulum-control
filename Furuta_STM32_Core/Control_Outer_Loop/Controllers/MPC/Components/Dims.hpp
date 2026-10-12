// Fixed compile-time dimensions of the MPC problem and the vector types
// built on them.
//
// These sizes come from the physical system and never change with tuning:
//   state  x = [theta, alpha, theta_dot, alpha_dot]   (alpha = 0 is upright)
//   output z = the full state (every state is measured or estimated)
//   input  u = arm torque (scalar)
//
// The horizon lengths HP and HU are tuning parameters, so they are not here.
// They are defined in the generated uMPC_Dependencies.hpp, next to the
// matrices whose sizes they determine.
#pragma once
#include <Eigen/Dense>

namespace furuta {
namespace mpc {

static constexpr int NX = 4; // states
static constexpr int NZ = 4; // outputs (full-state feedback)
static constexpr int NU = 1; // inputs (arm torque)

using StateVec = Eigen::Matrix<float, NX, 1>;
using InputVec = Eigen::Matrix<float, NU, 1>;

} // namespace mpc
} // namespace furuta
