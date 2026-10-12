// Switching the active controller from the debug CLI while running.
//
// CLI commands (hp_lqr, hp_smc, hp_umpc, hp_nn, hp_cmpc, hp_nmpc, sw_nn,
// sw_eng, sw_nmpc) write a request into one of the two mailboxes below. The
// outer loop reads and clears them once per tick and only applies them in the
// STOP state. Switching mid-run could hand a controller stale internal state,
// so the rule is the same as for starting a swing-up: only from STOP.
//
// Switching to a Pi-based controller also asks the Pi to reset that solver, so
// it starts clean instead of from a stale warm start.
#pragma once
#include <atomic>
#include <cstdint>
#include "../ControllerChoices.hpp"

namespace furuta
{

enum class HoldUpSwitchRequest
{
    NONE,
    LQR,
    SMC,
    UNCONSTRAINED_MPC,
    NN_HOLD_UP,
    CONSTRAINED_MPC,
    NMPC_STABILIZE,
};

enum class SwingUpSwitchRequest
{
    NONE,
    NN_SWING_UP,
    ENERGY_SWING_UP,
    NMPC_SWINGUP,
};

inline std::atomic<HoldUpSwitchRequest> g_hold_up_switch_request{HoldUpSwitchRequest::NONE};
inline std::atomic<SwingUpSwitchRequest> g_swing_up_switch_request{SwingUpSwitchRequest::NONE};

// What happened to a request. As with state requests, the outer loop runs in
// an interrupt and cannot print, so the outcome is published here.
enum class ControllerSwitchOutcome
{
    NONE,
    APPLIED,  // now running the requested controller
    ALREADY,  // that controller was already active
    REJECTED, // not in STOP; nothing changed
};

enum class ControllerFamily
{
    HOLD_UP,
    SWING_UP,
};

struct ControllerSwitchResult
{
    ControllerSwitchOutcome outcome = ControllerSwitchOutcome::NONE;
    ControllerFamily family = ControllerFamily::HOLD_UP;
    HoldUpController hold_up_value = HoldUpController::LQR;                // valid if family == HOLD_UP
    SwingUpController swing_up_value = SwingUpController::ENERGY_SWING_UP; // valid if family == SWING_UP
};

// The outer loop fills the result and then increments the sequence number;
// the CLI re-reads the result whenever the number changes.
inline ControllerSwitchResult g_controller_switch_result{};
inline std::atomic<uint32_t> g_controller_switch_result_seq{0};

} // namespace furuta
