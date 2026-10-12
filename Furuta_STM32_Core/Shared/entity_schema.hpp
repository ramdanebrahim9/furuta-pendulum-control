// What each logged channel (entity slot) is, per stream.
//
// The names are written into the header of every log file, so a decoder
// (MATLAB or Python) reads them from the file itself with nothing to keep in
// sync. When a slot is repurposed, update its name here and the index at the
// push site.
#pragma once
#include <array>
#include "CtrlLog.hpp"
#include "PendlLog.hpp"

struct EntityMeta
{
    char name[24];
};
static_assert(sizeof(EntityMeta) == 24, "unexpected padding in EntityMeta");

// Inner current loop, every 100 us. Pushed by control_loop_step().
inline constexpr std::array<EntityMeta, CtrlLog::N_ENTITIES> kCtrlSchema = {{
    {"i_ref"},  // current reference [A]
    {"i_raw"},  // measured current [A]
    {"u_cmd"},  // voltage command [V]
}};

// Outer pendulum loop, every 5 ms. Pushed by outer_loop_step().
inline constexpr std::array<EntityMeta, PendlLog::N_ENTITIES> kPendlSchema = {{
    {"arm_angle"},        // raw arm encoder angle [rad]
    {"pendulum_angle"},   // raw pendulum encoder angle [rad], 0 = down
    {"ekf_theta_dot"},    // estimated arm speed [rad/s]
    {"ekf_alpha_dot"},    // estimated pendulum speed [rad/s]
    {"ekf_d_r"},          // estimated arm disturbance torque
    {"ekf_d_p"},          // estimated pendulum disturbance torque
    {"theta_dot_filt"},   // arm speed after smoothing, as the controllers see it
    {"tau_from_current"}, // torque built from the motor current (estimator input)
    {"tau_cmd"},          // torque demand sent to the inner loop
    {"alpha_wrapped"},    // pendulum angle wrapped to [-pi, pi], 0 = upright
    {"ekf_theta"},        // estimated arm angle (true, unshifted)
    {"ekf_alpha"},        // estimated pendulum angle, unwrapped
    {"alpha_dot_filt"},   // pendulum speed after smoothing
    {"u_prev_meas"},      // torque measured on the motor
}};
