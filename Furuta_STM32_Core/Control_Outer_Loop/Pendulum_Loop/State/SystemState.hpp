// The states of the outer-loop state machine.
//
// The outer loop is a state machine that picks which controller runs each
// tick, depending on where the pendulum is.
//
//        +------------------------------+
//        v                              |
//     SWING_UP --(near upright)--> HOLD_UP --(falls out of window)--> HOMING
//        ^                                                              |
//        +-----------------------(arm centred)--------------------------+
//
//   SWING_UP  Pendulum hanging or swinging; pump energy towards the top.
//             Goes to HOLD_UP once alpha enters the catch window.
//   HOLD_UP   Pendulum near upright; a balance controller holds it there.
//             Goes to HOMING if the pendulum falls out of the window.
//   HOMING    Pendulum lost; drive the arm back to centre before the next
//             swing-up, so the cable does not wind up against the limit.
//             Goes to SWING_UP once the arm is centred.
//   STOP      Manual pause with zero torque. Entered from any state by the
//             debug CLI's "stop". Only from STOP can "go" start a new swing-up,
//             which allows launching from an arbitrary, disturbed pendulum
//             state. Controller switching is also only allowed in STOP.
//   DEAD      The arm passed its mechanical limit. Reachable from any state,
//             zero torque, and terminal: only an MCU reset leaves it.
//
// Every controller exposes the same compute(state) -> torque interface, so
// changing which one runs in a state never changes this enum.
#pragma once

namespace furuta
{

enum class SystemState
{
    SWING_UP,
    HOLD_UP,
    HOMING,
    DEAD,
    STOP,
};

} // namespace furuta
