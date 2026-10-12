// The outer (pendulum) control loop, one step every 5 ms.
//
// Each step reads the sensors, estimates the full state, runs the state
// machine, and publishes a torque demand for the fast inner current loop.
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    // One outer-loop step; called from the outer-loop timer interrupt.
    void outer_loop_step(void);

    // Clears per-run state; call before a new experiment starts.
    void outer_loop_reset(void);

    // Passes a received UART frame to whichever Pi-based controller is
    // waiting for it. Returns false if the frame was rejected (bad marker
    // byte), true otherwise, including when no Pi-based controller is active.
    // Called from the UART receive callback.
    bool remote_controller_on_uart_rx(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// C++ only: these return enum classes, which cannot cross the extern "C"
// boundary above. Used by the debug CLI to report status and the cause of death.
#include "State/DeathReason.hpp"
#include "State/SystemState.hpp"

furuta::DeathReason outer_loop_get_death_reason(void);
furuta::SystemState outer_loop_get_state(void);
#endif
