// Sliding-mode balance controller for the upright Furuta pendulum.
//
// Method: aggregated hierarchical sliding-mode control (Qian & Yi 2015,
// "Hierarchical Sliding Mode Control for Under-actuated Cranes", Sec 4.2).
//
// The Furuta pendulum has one motor and two degrees of freedom (arm and
// pendulum). Each degree of freedom gets its own sliding surface, and the two
// are combined into a single surface S that the one motor drives to zero.
// Once S = 0, both the arm and the pendulum settle at the upright rest point.
//
//   State  x = [theta, alpha, theta_dot, alpha_dot]   (alpha = 0 is upright)
//   Output u = arm torque [N.m], saturated to +-TAU_MAX
//
// Sliding surfaces, using the error e = -x (the reference is upright rest):
//   s1 = C1 * e_theta + edot_theta      arm        (actuated)
//   s2 = C2 * e_alpha + edot_alpha      pendulum   (unactuated)
//   S  = ALPHA_H * s1 + s2              combined surface
//
// Control law: an equivalent term that keeps the system on the surface, plus
// a reaching term that pulls it onto the surface from anywhere.
//   ueq1 = -(C1 * theta_dot + f_theta) / b_theta
//   ueq2 = -(C2 * alpha_dot + f_alpha) / b_alpha
//   den  = ALPHA_H * b_theta + b_alpha
//   u    = (ALPHA_H * b_theta * ueq1 + b_alpha * ueq2 + KAPPA * S + ETA * sw(S)) / den
// f and b are the plant's drift and input gain (see Components/furuta_get_fb.hpp).
// The switching function sw is tanh(S / PHI) rather than sign(S): the
// boundary layer of width PHI removes chattering.
//
// Reaching law: a proportional term (KAPPA * S) plus a small switching term
// (ETA * sw). Leaning on KAPPA and keeping ETA low cut chattering from about
// 990 to 2 sign flips per second in simulation, with alpha held within 0.1 deg.
//
// Model-based: unlike the LQR, the equivalent control cancels the plant
// dynamics, so it uses the same identified model as the rest of the system.
//
// Sign conventions worth keeping:
//   - ALPHA_H is negative. The denominator den must never reach zero, and
//     it does when ALPHA_H reaches a value set by the plant (the ratio of
//     the two input gains near upright), so ALPHA_H is kept well away from it.
//   - The rest reference gives edot = -x_dot, so the measured rates are used
//     directly in the equivalent control.
//
// Stateless: u depends only on the current state. Called at the outer-loop
// rate (200 Hz).
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "FurutaModel.hpp"   // furuta_params(): identified plant values
#include "furuta_get_fb.hpp" // plant drift and input gain

namespace furuta
{

class Controller_SMC
{
  public:
    static constexpr int NX = 4;
    using StateVec = Eigen::Matrix<float, NX, 1>;

    // Saturated arm torque.
    float compute(const StateVec& x) const
    {
        return clampf(computeRaw(x), TAU_MAX);
    }

    // Torque before saturation.
    float computeRaw(const StateVec& x) const
    {
        const float theta_dot = x(2);
        const float alpha_dot = x(3);

        // Tracking errors; the reference is upright rest (all zero).
        const float e_theta = -x(0);
        const float e_alpha = -x(1);
        const float edot_theta = -x(2);
        const float edot_alpha = -x(3);

        // Sub-surfaces and the combined surface.
        const float s1 = C1 * e_theta + edot_theta; // arm
        const float s2 = C2 * e_alpha + edot_alpha; // pendulum
        const float S = ALPHA_H * s1 + s2;

        // Plant drift f and input gain b from the identified model.
        Eigen::Matrix<float, 2, 1> q(x(0), x(1));
        Eigen::Matrix<float, 2, 1> qd(x(2), x(3));
        Eigen::Matrix<float, 2, 1> f, b;
        furuta_get_fb(q, qd, furuta_params(), f, b);
        const float f_theta = f(0), f_alpha = f(1);
        const float b_theta = b(0), b_alpha = b(1);

        // Equivalent control for each subsystem, then their weighted blend.
        const float ueq1 = -(C1 * theta_dot + f_theta) / b_theta;
        const float ueq2 = -(C2 * alpha_dot + f_alpha) / b_alpha;
        const float den = ALPHA_H * b_theta + b_alpha;
        const float u_eq = (ALPHA_H * b_theta * ueq1 + b_alpha * ueq2) / den;

        // Reaching term: tanh boundary layer, or plain sign if PHI <= 0.
        const float sw = (PHI > 0.0f) ? std::tanh(S / PHI) : signf(S);
        const float u_reach = (KAPPA * S + ETA * sw) / den;

        return u_eq + u_reach;
    }

    static constexpr float TAU_MAX = 5.0f; // torque limit [N.m]

    // Tuning, withheld in this public version (0.0f placeholders). The real
    // values come from the MATLAB simulation.
    static constexpr float C1 = 0.0f;      // arm surface slope (> 0)
    static constexpr float C2 = 0.0f;      // pendulum surface slope (> 0)
    static constexpr float ALPHA_H = 0.0f; // surface weighting, negative (see header)
    static constexpr float KAPPA = 0.0f;   // proportional reaching gain
    static constexpr float ETA = 0.0f;     // switching gain
    static constexpr float PHI = 0.0f;     // boundary-layer width (<= 0 selects sign)

  private:
    // Sign with 0 at exactly 0.
    static inline float signf(float v)
    {
        return (v > 0.0f) ? 1.0f : ((v < 0.0f) ? -1.0f : 0.0f);
    }
    // Symmetric clamp to +-lim.
    static inline float clampf(float v, float lim)
    {
        return (v > lim) ? lim : ((v < -lim) ? -lim : v);
    }
};

} // namespace furuta
