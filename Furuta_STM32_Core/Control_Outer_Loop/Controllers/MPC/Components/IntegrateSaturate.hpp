// MPC step 4: turn the solution into this tick's input.
//
// The solver returns a whole sequence of future increments. Only the first
// is applied (receding horizon); the rest are recomputed next tick. The
// increment is added to the previous input and clamped to the actuator
// limit:
//
//   u(k) = clamp( u(k-1) + dU(0),  -tau_max, +tau_max )
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include "Dims.hpp"
#include "BuildGradient.hpp"

namespace furuta {
namespace mpc_detail {

// u_prev  : input applied last tick
// dU      : full increment sequence from the solver
// tau_max : actuator limit [N.m]
template <int Hu>
inline mpc::InputVec integrateSaturate(const mpc::InputVec& u_prev,
                                       const DecisionVecT<Hu>& dU,
                                       float tau_max)
{
    const mpc::InputVec du0 = dU.template head<mpc::NU>(); // first move only
    mpc::InputVec u = u_prev + du0;

    for (int i = 0; i < mpc::NU; ++i)
        u(i) = std::max(-tau_max, std::min(tau_max, u(i)));

    return u;
}

} // namespace mpc_detail
} // namespace furuta
