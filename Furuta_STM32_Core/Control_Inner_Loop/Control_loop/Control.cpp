// Inner current loop, executed every 100 us (10 kHz).
//
//   outer loop torque demand --> CurrentRefBuilder --> i_ref
//                                                        |
//   current sensor --> EKF (i, disturbance) --> i_est --> PI --> PWM duty
//                                  ^                              |
//                                  +---------- u_prev ------------+
//
// The outer (balance) loop publishes a torque demand every 5 ms. This loop
// converts it to a current reference, estimates the motor current with a
// 2-state EKF, and closes the current with a PI controller.

#include "Control.hpp"
#include "CtrlLog.hpp"
#include "CausalLowpass.hpp"
#include "CurrentRefBuilder.hpp"
#include "EKF.hpp"
#include "MotorPI.hpp"
#include "SampleTimes.hpp"
#include "Sensors.hpp"
#include "current.hpp"
#include "pwm_engine.hpp"

extern CurrentSensor current;

// PI current controller. Kp and Ki come from pole-zero cancellation: the PI
// zero (Ki/Kp) cancels the motor's electrical pole (R/L), leaving a
// first-order closed loop.
static MotorPI g_pi(MotorPI::Config{
    .Kp = 0.0f, // dummy placeholder, real gain withheld
    .Ki = 0.0f, // dummy placeholder, real gain withheld
    .Ts = SAMPLE_TIME_INNER_S,
    .V_max = 10.0f,
    .Kb = 2.0f, // anti-windup back-calculation gain
});

// Light ripple filter on the measured current, run at the full 10 kHz rate.
// The outer loop samples it every 5 ms as the measured previous command, so
// the PWM ripple must be removed here or it aliases into that sample.
static constexpr float I_RIPPLE_FILTER_FC_HZ = 400.0f;
static CausalLowpass g_i_ripple_filter(I_RIPPLE_FILTER_FC_HZ, true, 1.0f / SAMPLE_TIME_INNER_S);

// Voltage applied on the previous tick; the EKF predicts with it.
static float u_prev = 0.0f;

void control_reset(void)
{
    g_pi.reset();
    EKF::init();
    g_i_ripple_filter.reset();
    u_prev = 0.0f;
}

void control_loop_step(void)
{
    const float i_meas = current.readCurrent();
    sensors_publish_current(i_meas);                                  // outer loop builds torque from this
    sensors_publish_current_filt(g_i_ripple_filter.step(i_meas));     // outer loop measured u_prev

    // Torque demand from the outer loop, converted to a current reference.
    const float i_ref = CurrentRefBuilder::step(sensors_read_tau());

    // State estimate [i, d]: motor current and lumped disturbance.
    const EKF_State& est = EKF::step(u_prev, i_meas, i_ref);
    const float i_est = est.x_est(0);

    const float u = g_pi.update(i_ref, i_est);
    PWMEngine::set_duty(u);

    u_prev = u;

    // Push this tick's current-loop signals into the log buffer.
    if constexpr (LOGGING_ENABLED)
    {
        CtrlLog::Sample sample{};
        sample.entity[0] = i_ref;
        sample.entity[1] = i_meas;
        sample.entity[2] = u;
        CtrlLog::FillBuffer(sample);
    }
}
