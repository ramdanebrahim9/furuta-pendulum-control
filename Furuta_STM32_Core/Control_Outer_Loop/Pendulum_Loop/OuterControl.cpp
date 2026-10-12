// The outer control loop, run every 5 ms from a timer interrupt.
//
// One step does:
//   1. read the encoders and the motor current published by the inner loop
//   2. estimate the full state (FurutaEKF)
//   3. handle manual commands and update the state machine
//   4. compute the torque with whichever controller the state calls for
//   5. publish the torque demand for the inner current loop, and log
//
// The controllers themselves live in ControllerBank; the rules for moving
// between states live in StateMachine.

#include "OuterControl.hpp"
#include <cmath>
#include "Commands/ArmReference.hpp"
#include "Commands/ControllerSwitch.hpp"
#include "Commands/StateCommand.hpp"
#include "CausalLowpass.hpp"
#include "ControllerBank.hpp"
#include "ControllerChoices.hpp"
#include "FurutaConventions.hpp"
#include "FurutaEKF.hpp"
#include "log_config.hpp" // loop rates and LOGGING_ENABLED
#include "Sensors.hpp"
#include "StateMachine.hpp"
#include "TauBuilder.hpp"
#include "UPrevBuilder.hpp"
#include "PendlLog.hpp"

// Raw motor current to arm torque, as the estimator's input.
static TauBuilder g_tau_builder;

// Torque actually measured on the motor, used by the velocity-form MPCs.
static UPrevBuilder g_u_prev_builder;

// Safety and hand-off limits
// The arm must stay inside its mechanical limit (the cable twists beyond it);
// leaving it ends the run (DEAD).
static constexpr float ARM_LIMIT_RAD = 355.0f * static_cast<float>(M_PI) / 180.0f;
// How close to upright the pendulum must be for SWING_UP to hand over to
// HOLD_UP, and for HOLD_UP to keep running.
static constexpr float CATCH_WINDOW_RAD = 45.0f * static_cast<float>(M_PI) / 180.0f;

// Smoothing of the estimated speeds before the controllers see them.
static constexpr float THETA_DOT_FILTER_HZ = 8.0f;  // arm speed
static constexpr float ALPHA_DOT_FILTER_HZ = 40.0f; // pendulum speed
static CausalLowpass g_theta_dot_filter(THETA_DOT_FILTER_HZ, true);
static CausalLowpass g_alpha_dot_filter(ALPHA_DOT_FILTER_HZ, true);

void outer_loop_reset(void)
{
    FurutaEKF::init();
    g_tau_builder.reset();
    g_u_prev_builder.reset();
    g_theta_dot_filter.reset();
    g_alpha_dot_filter.reset();

    furuta::bank::reset(furuta::g_swing_up_choice.load(std::memory_order_relaxed));
    furuta::state_machine::reset();
}

void outer_loop_step(void)
{
    // 1. Sensors. The encoder angles come from one consistent sample taken by
    // the fast loop.
    const SensorAngles angles = sensors_read_angles();
    const float tau_from_current = g_tau_builder.step(sensors_read_current());
    const float u_prev_meas = g_u_prev_builder.step();

    // 2. State estimate: [theta, alpha, theta_dot, alpha_dot, d_r, d_p].
    const FurutaEKF_State& est = FurutaEKF::step(tau_from_current, angles.arm_rad, angles.pendl_rad);

    // 3. The state the controllers see.
    //   - Speeds are smoothed: a noisy speed would reach the torque directly.
    //   - alpha is wrapped to [-pi, pi]: the controllers are designed around
    //     alpha = 0 (upright), and the estimate keeps counting past a full turn.
    //   - theta is shifted by the live arm-centre setting ("trj" command). Only
    //     the controllers see the shift; the safety check and the log use the
    //     true arm angle.
    const float theta_dot = g_theta_dot_filter.step(est.x_est(2));
    const float alpha_dot = g_alpha_dot_filter.step(est.x_est(3));
    const float alpha_wrapped = wrap_pi(est.x_est(1));
    const float theta_shifted =
        est.x_est(0) - furuta::g_arm_reference_shift_rad.load(std::memory_order_relaxed);

    const furuta::bank::StateVec x_ctrl{theta_shifted, alpha_wrapped, theta_dot, alpha_dot};

    // 4. Manual commands, then the state machine.
    // Requests are read and cleared once per tick. Controller switches are
    // applied first, so their STOP-only rule sees the state from the start of
    // the tick; the selections are read afterwards.
    const furuta::SystemStateRequest state_request =
        furuta::g_system_state_request.exchange(furuta::SystemStateRequest::NONE, std::memory_order_relaxed);
    const furuta::HoldUpSwitchRequest hold_up_request =
        furuta::g_hold_up_switch_request.exchange(furuta::HoldUpSwitchRequest::NONE, std::memory_order_relaxed);
    const furuta::SwingUpSwitchRequest swing_up_request =
        furuta::g_swing_up_switch_request.exchange(furuta::SwingUpSwitchRequest::NONE, std::memory_order_relaxed);

    furuta::state_machine::apply_controller_switch(hold_up_request, swing_up_request);

    const furuta::HoldUpController hold_up = furuta::g_hold_up_choice.load(std::memory_order_relaxed);
    const furuta::SwingUpController swing_up = furuta::g_swing_up_choice.load(std::memory_order_relaxed);

    furuta::state_machine::apply_state_request(state_request, swing_up);

    // The safety check uses the true arm angle. The arm cannot complete a
    // turn (cable), so the angle is not wrapped.
    furuta::state_machine::update({
        std::fabs(est.x_est(0)) <= ARM_LIMIT_RAD,
        std::fabs(alpha_wrapped) <= CATCH_WINDOW_RAD,
        furuta::bank::arm_is_homed(x_ctrl),
        hold_up,
        swing_up,
    });

    // 5. Torque from the controller that the state selects, handed to the
    // inner loop.
    const float tau_cmd = furuta::bank::compute(furuta::state_machine::state(), swing_up, hold_up,
                                                x_ctrl, u_prev_meas);
    sensors_publish_tau(tau_cmd);

    // 6. Log. Keep in sync with kPendlSchema in Shared/entity_schema.hpp.
    if constexpr (LOGGING_ENABLED)
    {
        PendlLog::Sample sample{};
        sample.entity[0] = angles.arm_rad;
        sample.entity[1] = angles.pendl_rad; // raw encoder (0 = down, pi = up)
        sample.entity[2] = est.x_est(2);     // estimated speeds, before smoothing
        sample.entity[3] = est.x_est(3);
        sample.entity[4] = est.x_est(4);     // disturbance estimates d_r, d_p
        sample.entity[5] = est.x_est(5);
        sample.entity[6] = theta_dot;        // smoothed speeds
        sample.entity[7] = tau_from_current; // torque built from the motor current
        sample.entity[8] = tau_cmd;          // torque demand sent to the inner loop
        sample.entity[9] = alpha_wrapped;    // alpha wrapped to [-pi, pi]
        sample.entity[10] = est.x_est(0);    // theta, true (unshifted)
        sample.entity[11] = est.x_est(1);    // alpha, unwrapped
        sample.entity[12] = alpha_dot;
        sample.entity[13] = u_prev_meas;     // torque measured on the motor
        PendlLog::FillBuffer(sample);
    }
}

bool remote_controller_on_uart_rx(void)
{
    return furuta::bank::on_uart_rx(furuta::state_machine::state(),
                                    furuta::g_swing_up_choice.load(std::memory_order_relaxed),
                                    furuta::g_hold_up_choice.load(std::memory_order_relaxed));
}

furuta::DeathReason outer_loop_get_death_reason(void)
{
    return furuta::state_machine::death_reason();
}

furuta::SystemState outer_loop_get_state(void)
{
    return furuta::state_machine::state();
}
