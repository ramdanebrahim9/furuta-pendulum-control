// MPC step 3: solve for the optimal input increments.
//
// Without constraints the minimum of the cost dU'*H*dU - G'*dU has a
// closed form:
//
//   2 * H * dU = G   ->   dU = 0.5 * H^-1 * G
//
// H^-1 is computed offline (Hinv), so a tick costs one fixed-size
// matrix-vector product: no iteration, no factorisation, fixed run time.
// This is the only step that would change for a constrained MPC, which
// would solve a QP here instead.
#pragma once
#include <Eigen/Dense>
#include "Dims.hpp"
#include "BuildGradient.hpp"

namespace furuta {
namespace mpc_detail {

// Hinv : inverse Hessian (generated offline)
// G    : gradient from buildGradient()
template <int Hu>
inline DecisionVecT<Hu> solveUnconstrained(const Eigen::Matrix<float, Hu*mpc::NU, Hu*mpc::NU>& Hinv,
                                           const DecisionVecT<Hu>& G)
{
    return 0.5f * (Hinv * G);
}

} // namespace mpc_detail
} // namespace furuta
