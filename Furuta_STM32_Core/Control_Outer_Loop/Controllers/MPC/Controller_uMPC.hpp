// Unconstrained MPC balance controller (velocity form) for the upright pendulum.
//
// Same interface as the other balance controllers, except it also needs the
// torque currently on the motor:  u = ctrl.compute(x, u_prev).
//
//   State  x = [theta, alpha, theta_dot, alpha_dot]   (alpha = 0 is upright)
//   Output u = arm torque [N.m], saturated to +-TAU_MAX
//
// Velocity form: instead of the torque itself, the optimiser chooses the
// torque *increments* over a control horizon, and the first one is added to
// the previous torque: u(k) = u(k-1) + dU(0). This gives integral action and
// makes the controller insensitive to constant offsets.
//
// u(k-1) is the torque actually measured on the motor (from the filtered
// current), not the last command. The MPC therefore always integrates onto
// what the plant really received. The class holds no state.
//
// One call runs four small steps (see Components/):
//   BuildError -> BuildGradient -> SolveUnconstrained -> IntegrateSaturate
//
// The prediction matrices and the inverse Hessian depend on the weights and
// horizons, so they are generated offline (Dependencies/uMPC_Dependencies.hpp).
// Retuning means regenerating that file; this class does not change.
//
// The matrices assume compute() is called once per TS.
#pragma once
#include <Eigen/Dense>
#include "Components/BuildError.hpp"
#include "Components/BuildGradient.hpp"
#include "Components/Dims.hpp"
#include "Components/IntegrateSaturate.hpp"
#include "Components/SolveUnconstrained.hpp"
#include "Dependencies/uMPC_Dependencies.hpp"

namespace furuta {

class Controller_uMPC
{
  public:
    static constexpr int NX = mpc::NX;
    using StateVec = mpc::StateVec;

    static constexpr float TAU_MAX = 5.0f; // torque limit [N.m]
    static constexpr float U_EQ = 0.0f;    // equilibrium torque [N.m]
    static constexpr float TS = 0.005f;    // sample time the model was built for [s]

    // Saturated arm torque.
    // x      : state
    // u_prev : torque measured on the motor this tick [N.m]
    float compute(const StateVec& x, float u_prev) const
    {
        const mpc::InputVec u = mpc_detail::integrateSaturate<mpc_gen::HU>(
            mpc::InputVec::Constant(u_prev), solveIncrements(x, u_prev), TAU_MAX);
        return u(0);
    }

    // Torque before saturation.
    float computeRaw(const StateVec& x, float u_prev) const
    {
        return u_prev + solveIncrements(x, u_prev)(0);
    }

  private:
    // Steps 1-3: optimal increment sequence dU for the current state.
    static mpc_detail::DecisionVecT<mpc_gen::HU> solveIncrements(const StateVec& x, float u_prev)
    {
        const auto& D = deps();
        const mpc::InputVec u_prev_v = mpc::InputVec::Constant(u_prev);

        // Error to the upright set-point if no further move were made.
        const auto E = mpc_detail::buildError<mpc_gen::HP>(D.CPSI(), D.COMEGA(), x, u_prev_v, upright_setpoint());

        // Cost gradient for that error.
        const auto G = mpc_detail::buildGradient<mpc_gen::HP, mpc_gen::HU>(D.CTHETA_Q(), E);

        // Minimiser of the cost.
        return mpc_detail::solveUnconstrained<mpc_gen::HU>(D.Hinv(), G);
    }

    // Generated matrices, built once on first use.
    static const mpc_gen::MPC_Dependencies& deps()
    {
        static const mpc_gen::MPC_Dependencies d{};
        return d;
    }

    // Set-point over the horizon: all zeros (upright, arm centred).
    static const mpc_detail::HorizonVecT<mpc_gen::HP>& upright_setpoint()
    {
        static const mpc_detail::HorizonVecT<mpc_gen::HP> sp = mpc_detail::HorizonVecT<mpc_gen::HP>::Zero();
        return sp;
    }
};

} // namespace furuta
