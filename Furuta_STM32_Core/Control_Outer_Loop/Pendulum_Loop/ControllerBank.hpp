// All controller instances, and the single place that maps a state (and the
// selected controller) to the torque command.
//
// The outer loop knows nothing about individual controllers: it hands the
// current state, the selection and the estimated system state to compute()
// and gets a torque back. Every controller has the same compute(state)
// interface, so adding or swapping one only touches ControllerBank.cpp.
#pragma once
#include <Eigen/Dense>
#include "ControllerChoices.hpp"
#include "State/SystemState.hpp"

namespace furuta
{
namespace bank
{

// [theta, alpha, theta_dot, alpha_dot], alpha = 0 is upright.
using StateVec = Eigen::Matrix<float, 4, 1>;

// Clears the Pi link state of the remote controllers at the start of a run.
// The run starts in SWING_UP, so a Pi swing-up solver also gets a cold start.
void reset(SwingUpController swing_up);

// Torque [N.m] for the current state. Zero in STOP and DEAD.
//   x      : state as seen by the controllers
//   u_prev : torque measured on the motor this tick
float compute(SystemState state, SwingUpController swing_up, HoldUpController hold_up,
              const StateVec& x, float u_prev);

// True once the arm is centred, which ends HOMING.
bool arm_is_homed(const StateVec& x);

// Ask the Pi to reset the solver behind this controller before its next use,
// so it starts clean instead of from a stale warm start. No effect for
// controllers that run on the STM32.
void request_pi_reset(SwingUpController swing_up);
void request_pi_reset(HoldUpController hold_up);

// Hands a received UART frame to the Pi-based controller that is waiting for
// it. Returns false if the frame was rejected, true otherwise (including when
// no Pi-based controller is active).
bool on_uart_rx(SystemState state, SwingUpController swing_up, HoldUpController hold_up);

} // namespace bank
} // namespace furuta
