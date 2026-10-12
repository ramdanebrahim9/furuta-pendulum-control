#include "StateMachine.hpp"
#include "ControllerBank.hpp"
#include "Controller_Remote.hpp"

namespace furuta
{
namespace state_machine
{

static SystemState g_state = SystemState::SWING_UP;
static DeathReason g_death_reason = DeathReason::SYSTEM_ALIVE;

SystemState state() { return g_state; }
DeathReason death_reason() { return g_death_reason; }

void reset()
{
    // DEAD is terminal: starting a new run must not revive it.
    if (g_state != SystemState::DEAD)
        g_state = SystemState::SWING_UP;

    // A request typed just before the start (a stray "go") must not leak into
    // the new run, which always begins in SWING_UP.
    g_system_state_request.store(SystemStateRequest::NONE, std::memory_order_relaxed);
}

// ── controller switching ────────────────────────────────────────────────
// Neither handler may print: this runs in an interrupt. They publish the
// outcome for the CLI instead.

static void publish_switch_result(ControllerSwitchOutcome outcome, ControllerFamily family,
                                  HoldUpController hold_up, SwingUpController swing_up)
{
    g_controller_switch_result = {outcome, family, hold_up, swing_up};
    g_controller_switch_result_seq.fetch_add(1, std::memory_order_relaxed);
}

static HoldUpController to_controller(HoldUpSwitchRequest r)
{
    switch (r)
    {
    case HoldUpSwitchRequest::LQR: return HoldUpController::LQR;
    case HoldUpSwitchRequest::SMC: return HoldUpController::SMC;
    case HoldUpSwitchRequest::UNCONSTRAINED_MPC: return HoldUpController::UNCONSTRAINED_MPC;
    case HoldUpSwitchRequest::NN_HOLD_UP: return HoldUpController::NN_HOLD_UP;
    case HoldUpSwitchRequest::CONSTRAINED_MPC: return HoldUpController::CONSTRAINED_MPC;
    case HoldUpSwitchRequest::NMPC_STABILIZE: return HoldUpController::NMPC_STABILIZE;
    case HoldUpSwitchRequest::NONE: break;
    }
    return HoldUpController::LQR;
}

static SwingUpController to_controller(SwingUpSwitchRequest r)
{
    switch (r)
    {
    case SwingUpSwitchRequest::NN_SWING_UP: return SwingUpController::NN_SWING_UP;
    case SwingUpSwitchRequest::ENERGY_SWING_UP: return SwingUpController::ENERGY_SWING_UP;
    case SwingUpSwitchRequest::NMPC_SWINGUP: return SwingUpController::NMPC_SWINGUP;
    case SwingUpSwitchRequest::NONE: break;
    }
    return SwingUpController::ENERGY_SWING_UP;
}

void apply_controller_switch(HoldUpSwitchRequest hold_up, SwingUpSwitchRequest swing_up)
{
    if (hold_up != HoldUpSwitchRequest::NONE)
    {
        const HoldUpController target = to_controller(hold_up);
        ControllerSwitchOutcome outcome;

        if (g_state != SystemState::STOP)
            outcome = ControllerSwitchOutcome::REJECTED;
        else if (g_hold_up_choice.load(std::memory_order_relaxed) == target)
            outcome = ControllerSwitchOutcome::ALREADY;
        else
        {
            g_hold_up_choice.store(target, std::memory_order_relaxed);
            bank::request_pi_reset(target); // a Pi solver starts cold
            outcome = ControllerSwitchOutcome::APPLIED;
        }
        publish_switch_result(outcome, ControllerFamily::HOLD_UP, target, SwingUpController::ENERGY_SWING_UP);
    }

    if (swing_up != SwingUpSwitchRequest::NONE)
    {
        const SwingUpController target = to_controller(swing_up);
        ControllerSwitchOutcome outcome;

        if (g_state != SystemState::STOP)
            outcome = ControllerSwitchOutcome::REJECTED;
        else if (g_swing_up_choice.load(std::memory_order_relaxed) == target)
            outcome = ControllerSwitchOutcome::ALREADY;
        else
        {
            g_swing_up_choice.store(target, std::memory_order_relaxed);
            bank::request_pi_reset(target);
            outcome = ControllerSwitchOutcome::APPLIED;
        }
        publish_switch_result(outcome, ControllerFamily::SWING_UP, HoldUpController::LQR, target);
    }
}

// ── manual state requests ───────────────────────────────────────────────

void apply_state_request(SystemStateRequest request, SwingUpController swing_up)
{
    if (request == SystemStateRequest::NONE)
        return;

    SystemStateOutcome outcome;

    if (g_state == SystemState::DEAD)
    {
        outcome = SystemStateOutcome::REJECTED; // terminal
    }
    else if (request == SystemStateRequest::STOP)
    {
        if (g_state == SystemState::STOP)
        {
            outcome = SystemStateOutcome::ALREADY;
        }
        else
        {
            // Tell the Pi to stand down on every real transition into STOP,
            // whichever controller was running, since it may still be
            // mid-solve from an earlier selection.
            Controller_Remote::sendPiStopAll();
            g_state = SystemState::STOP;
            outcome = SystemStateOutcome::APPLIED;
        }
    }
    else // SWING_UP ("go"): only legal from STOP
    {
        if (g_state != SystemState::STOP)
        {
            outcome = SystemStateOutcome::REJECTED;
        }
        else
        {
            // The pendulum may be anywhere after STOP, so the swing-up
            // controller starts cold.
            g_state = SystemState::SWING_UP;
            bank::request_pi_reset(swing_up);
            outcome = SystemStateOutcome::APPLIED;
        }
    }

    g_system_state_result = {outcome, (request == SystemStateRequest::STOP) ? SystemState::STOP
                                                                            : SystemState::SWING_UP};
    g_system_state_result_seq.fetch_add(1, std::memory_order_relaxed);
}

// ── automatic transitions ───────────────────────────────────────────────

void update(const Conditions& c)
{
    // DEAD is checked first and from every state: the arm limit overrides
    // everything, including manual requests.
    if (!c.arm_ok)
    {
        g_death_reason = DeathReason::ARM_LIMIT;
        g_state = SystemState::DEAD;
        return;
    }

    switch (g_state)
    {
    case SystemState::SWING_UP:
        if (c.pendulum_near_upright)
        {
            g_state = SystemState::HOLD_UP;
            // Every entry into HOLD_UP, not just the first, resets a Pi
            // solver: after a fall its warm start no longer fits.
            bank::request_pi_reset(c.hold_up);
        }
        break;

    case SystemState::HOLD_UP:
        if (!c.pendulum_near_upright)
            g_state = SystemState::HOMING;
        break;

    case SystemState::HOMING:
        if (c.arm_homed)
        {
            g_state = SystemState::SWING_UP;
            bank::request_pi_reset(c.swing_up); // a new swing-up attempt starts cold
        }
        break;

    case SystemState::STOP: // only manual requests leave STOP
    case SystemState::DEAD: // terminal
        break;
    }
}

} // namespace state_machine
} // namespace furuta
