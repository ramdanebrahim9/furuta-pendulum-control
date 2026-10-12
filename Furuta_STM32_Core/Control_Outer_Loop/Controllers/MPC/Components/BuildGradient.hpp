// MPC step 2: the gradient of the cost function.
//
// The cost is quadratic in the future input increments dU. Its linear term
// depends on the predicted error E, so it must be rebuilt every tick:
//
//   G = 2 * CTHETA' * Q * E
//
// CTHETA' * Q is constant and precomputed offline (CTHETA_Q), so each tick
// is a single matrix-vector product.
#pragma once
#include <Eigen/Dense>
#include "Dims.hpp"
#include "BuildError.hpp"

namespace furuta {
namespace mpc_detail {

// Stacked vector of future input increments (Hu moves, NU inputs each).
template <int Hu>
using DecisionVecT = Eigen::Matrix<float, Hu * mpc::NU, 1>;

// CTHETA_Q : precomputed CTHETA' * Q
// E        : predicted error from buildError()
template <int Hp, int Hu>
inline DecisionVecT<Hu> buildGradient(const Eigen::Matrix<float, Hu*mpc::NU, Hp*mpc::NZ>& CTHETA_Q,
                                      const HorizonVecT<Hp>& E)
{
    return 2.0f * (CTHETA_Q * E);
}

} // namespace mpc_detail
} // namespace furuta
