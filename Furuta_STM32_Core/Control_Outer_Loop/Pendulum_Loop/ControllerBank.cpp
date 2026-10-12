#include "ControllerBank.hpp"
#include "Controller_Energy_SwingUp.hpp"
#include "Controller_Home.hpp"
#include "Controller_LQR.hpp"
#include "Controller_NN_HoldUp.hpp"
#include "Controller_NN_SwingUp.hpp"
#include "Controller_Remote.hpp"
#include "Controller_SMC.hpp"
#include "Controller_uMPC.hpp"
#include "log_config.hpp" // loop rates and LOGGING_ENABLED

namespace furuta
{
namespace bank
{

// Swing-up controllers
static Controller_Energy_SwingUp energy_swing_up;
static Controller_NN_SwingUp nn_swing_up;

// The Pi swing-up solver needs a fixed zero-torque hold after every reset,
// while it solves its first problem from cold. The Pi plans from "the state
// this long after the reset, under zero torque", so the hold must be a fixed
// length, never "until the reply arrives". It must match the Pi's setting.
static constexpr uint32_t NMPC_SWINGUP_COLD_HOLD_US = 115000;
static_assert(NMPC_SWINGUP_COLD_HOLD_US % PENDL_INTERVAL_US == 0,
              "the cold-start hold must be a whole number of outer-loop ticks");
static constexpr uint16_t NMPC_SWINGUP_COLD_HOLD_TICKS = NMPC_SWINGUP_COLD_HOLD_US / PENDL_INTERVAL_US;

static Controller_Remote remote_nmpc_swingup(Controller_Remote::SolverMode::NMPC_Swingup,
                                             NMPC_SWINGUP_COLD_HOLD_TICKS);

// Hold-up controllers
static Controller_LQR lqr;
static Controller_SMC smc;
static Controller_uMPC umpc;
static Controller_NN_HoldUp nn_hold_up;
static Controller_Remote remote_cmpc(Controller_Remote::SolverMode::cMPC);
static Controller_Remote remote_nmpc_stabilize(Controller_Remote::SolverMode::NMPC_Stabilize);

// Homing
static Controller_Home home;

void reset(SwingUpController swing_up)
{
    remote_nmpc_swingup.reset();
    remote_cmpc.reset();
    remote_nmpc_stabilize.reset();

    // The run starts in SWING_UP, which for the Pi solver is a cold start.
    request_pi_reset(swing_up);
}

float compute(SystemState state, SwingUpController swing_up, HoldUpController hold_up,
              const StateVec& x, float u_prev)
{
    switch (state)
    {
    case SystemState::SWING_UP:
        switch (swing_up)
        {
        case SwingUpController::NMPC_SWINGUP: return remote_nmpc_swingup.compute(x, u_prev);
        case SwingUpController::NN_SWING_UP: return nn_swing_up.compute(x);
        case SwingUpController::ENERGY_SWING_UP: return energy_swing_up.compute(x);
        }
        break;

    case SystemState::HOLD_UP:
        switch (hold_up)
        {
        case HoldUpController::LQR: return lqr.compute(x);
        case HoldUpController::SMC: return smc.compute(x);
        case HoldUpController::UNCONSTRAINED_MPC: return umpc.compute(x, u_prev);
        case HoldUpController::NN_HOLD_UP: return nn_hold_up.compute(x, u_prev);
        case HoldUpController::CONSTRAINED_MPC: return remote_cmpc.compute(x, u_prev);
        case HoldUpController::NMPC_STABILIZE: return remote_nmpc_stabilize.compute(x, u_prev);
        }
        break;

    case SystemState::HOMING:
        return home.compute(x);

    case SystemState::STOP:
    case SystemState::DEAD:
        break;
    }
    return 0.0f;
}

bool arm_is_homed(const StateVec& x)
{
    return Controller_Home::isSettled(x);
}

void request_pi_reset(SwingUpController swing_up)
{
    if (swing_up == SwingUpController::NMPC_SWINGUP)
        remote_nmpc_swingup.requestPiReset();
}

void request_pi_reset(HoldUpController hold_up)
{
    if (hold_up == HoldUpController::CONSTRAINED_MPC)
        remote_cmpc.requestPiReset();
    else if (hold_up == HoldUpController::NMPC_STABILIZE)
        remote_nmpc_stabilize.requestPiReset();
}

// SWING_UP and HOLD_UP never run together, and each picks at most one
// Pi-based controller, so at most one instance is waiting for a reply.
bool on_uart_rx(SystemState state, SwingUpController swing_up, HoldUpController hold_up)
{
    if (state == SystemState::SWING_UP)
    {
        if (swing_up == SwingUpController::NMPC_SWINGUP)
            return remote_nmpc_swingup.onUartRxComplete();
    }
    else if (state == SystemState::HOLD_UP)
    {
        if (hold_up == HoldUpController::CONSTRAINED_MPC)
            return remote_cmpc.onUartRxComplete();
        if (hold_up == HoldUpController::NMPC_STABILIZE)
            return remote_nmpc_stabilize.onUartRxComplete();
    }
    return true;
}

} // namespace bank
} // namespace furuta
