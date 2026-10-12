// Which controller runs in each state.
//
// State/SystemState.hpp says when (which phase we are in); this file says
// which implementation handles that phase.
//
//   HOLD_UP (upright balance)
//     LQR                linear state feedback
//     SMC                hierarchical sliding-mode control
//     UNCONSTRAINED_MPC  velocity-form MPC with a precomputed solution
//     NN_HOLD_UP         neural network cloned from the nonlinear MPC
//     CONSTRAINED_MPC    constrained MPC solved on the Raspberry Pi
//     NMPC_STABILIZE     nonlinear MPC solved on the Raspberry Pi
//
//   SWING_UP (pumping energy to reach upright)
//     ENERGY_SWING_UP    energy-based swing-up with arm centering
//     NN_SWING_UP        neural network cloned from the swing-up nonlinear MPC
//     NMPC_SWINGUP       nonlinear MPC solved on the Raspberry Pi
//
//   HOMING always runs Controller_Home. DEAD and STOP run no controller.
//
// The three Pi-based options share one class, Controller_Remote, and differ
// only in which solver the Pi runs (see Controllers/Remote_Controllers/).
//
// The selection is a runtime value, so controllers can be changed from the
// debug CLI without rebuilding (only while in STOP). The atomics are written
// from the CLI and read from the outer-loop interrupt.
#pragma once
#include <atomic>

namespace furuta
{

enum class HoldUpController
{
    LQR,
    UNCONSTRAINED_MPC,
    CONSTRAINED_MPC,
    NMPC_STABILIZE,
    SMC,
    NN_HOLD_UP,
};

enum class SwingUpController
{
    ENERGY_SWING_UP,
    NMPC_SWINGUP,
    NN_SWING_UP,
};

// Controllers used at start-up.
inline std::atomic<HoldUpController> g_hold_up_choice{HoldUpController::CONSTRAINED_MPC};
inline std::atomic<SwingUpController> g_swing_up_choice{SwingUpController::NN_SWING_UP};

} // namespace furuta
