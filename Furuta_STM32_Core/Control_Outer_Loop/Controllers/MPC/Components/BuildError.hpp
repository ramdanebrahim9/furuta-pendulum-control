// MPC step 1: the predicted tracking error over the horizon.
//
// This is how far the outputs would be from the set-point if no new control
// move were made: only the system's free response to the current state and
// the input already being applied.
//
//   E = SP - CPSI * x(k) - COMEGA * u(k-1)
//
// The sizes depend on the prediction horizon Hp, so this is a template. Each
// controller instantiates it with its own horizon, and the compiler checks
// that all matrix sizes agree.
#pragma once
#include <Eigen/Dense>
#include "Dims.hpp"

namespace furuta {
namespace mpc_detail {

// Stacked vector over the prediction horizon (Hp steps, NZ outputs each).
template <int Hp>
using HorizonVecT = Eigen::Matrix<float, Hp * mpc::NZ, 1>;

// CPSI   : free response of the outputs to the current state
// COMEGA : response to holding the previous input
// x      : current state
// u_prev : input applied last tick
// SP     : stacked set-point over the horizon
template <int Hp>
inline HorizonVecT<Hp> buildError(const Eigen::Matrix<float, Hp*mpc::NZ, mpc::NX>& CPSI,
                                  const Eigen::Matrix<float, Hp*mpc::NZ, mpc::NU>& COMEGA,
                                  const mpc::StateVec& x,
                                  const mpc::InputVec& u_prev,
                                  const HorizonVecT<Hp>& SP)
{
    return SP - CPSI * x - COMEGA * u_prev;
}

} // namespace mpc_detail
} // namespace furuta
