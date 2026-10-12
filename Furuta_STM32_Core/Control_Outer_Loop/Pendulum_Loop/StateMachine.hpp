// The outer-loop state machine: owns the current state, the reason for death,
// and the rules for moving between states.
//
// Three things change the state:
//   1. Automatic transitions from the sensors (see State/SystemState.hpp).
//   2. Manual "stop" / "go" requests from the debug CLI.
//   3. Controller switch requests, which are only honoured in STOP.
//
// Everything here runs inside the outer-loop interrupt, once per tick.
#pragma once
#include "ControllerChoices.hpp"
#include "Commands/ControllerSwitch.hpp"
#include "Commands/StateCommand.hpp"
#include "State/DeathReason.hpp"
#include "State/SystemState.hpp"

namespace furuta
{
namespace state_machine
{

SystemState state();
DeathReason death_reason();

// Called at the start of every run. Restarts in SWING_UP and drops any
// pending manual request. DEAD and its reason are left alone: they survive
// until an MCU reset.
void reset();

// Handles controller switch requests (hold-up and swing-up). Applied only in
// STOP. Call before apply_state_request(), so the rule is checked against the
// state as it was at the start of the tick.
void apply_controller_switch(HoldUpSwitchRequest hold_up, SwingUpSwitchRequest swing_up);

// Handles a manual "stop" / "go" request and publishes the outcome.
void apply_state_request(SystemStateRequest request, SwingUpController swing_up);

// What the transitions need to know this tick.
struct Conditions
{
    bool arm_ok;               // arm is inside its mechanical limit
    bool pendulum_near_upright; // pendulum is inside the catch window
    bool arm_homed;            // arm is centred (ends HOMING)
    HoldUpController hold_up;
    SwingUpController swing_up;
};

// Automatic transitions. Runs before the controller computes, so a state
// change and its first torque land in the same tick.
void update(const Conditions& c);

} // namespace state_machine
} // namespace furuta
