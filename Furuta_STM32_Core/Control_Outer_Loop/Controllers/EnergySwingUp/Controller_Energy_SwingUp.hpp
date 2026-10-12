// Energy-based swing-up controller.
//
// Method: Aström & Furuta 1996, "Swinging up a pendulum by energy control".
//
// Starting from hanging at rest, the arm pumps energy into the pendulum until
// its mechanical energy equals its energy at the upright position. At that
// point the pendulum is on the trajectory that passes through the top, and it
// arrives upright slowly enough for the hold-up controller to catch it.
//
//   State  x = [theta, alpha, theta_dot, alpha_dot]   (alpha = 0 is upright)
//   Output u = arm torque [N.m], saturated to +-TAU_MAX
//
// Energy pump: a torque proportional to the energy error, applied in the
// direction that increases the pendulum's energy.
//
//   E       = 0.5 * Jp_pivot * alpha_dot^2 + m*g*l * (cos(alpha) - 1)
//   E_err   = E - E0                 E0 = 0 at the top, negative below
//   tau_pump = clamp( K_ENERGY * E_err * dir,  +-TAU_SWINGUP )
//
// Arm centering: a soft PD on the arm angle so the arm does not wind past the
// cable limit before the pendulum reaches the top. It is added after the pump
// is clamped, so it keeps its own authority up to TAU_MAX.
//
//   tau = tau_pump - K_ARM_P * theta - K_ARM_D * theta_dot
//
// Sign convention: in this build's geometry the arm-pendulum coupling has the
// opposite sign to the textbook Furuta pendulum, so the energy-increasing
// direction is dir = -sign(alpha_dot * cos(alpha)). With the textbook sign
// the pendulum is pumped the wrong way and never leaves the bottom.
//
// Only cos(alpha) is used, so wrapped and unwrapped alpha both work.
// At the exact bottom (alpha_dot = 0) the pump is zero; the arm centering
// keeps running, and any small disturbance starts the swing.
//
// Stateless: u depends only on the current state. Called at the outer-loop
// rate (200 Hz).
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "FurutaModel.hpp" // furuta_params(): identified plant values

namespace furuta
{

class Controller_Energy_SwingUp
{
  public:
    static constexpr int NX = 4;
    using StateVec = Eigen::Matrix<float, NX, 1>;

    static constexpr float TAU_MAX = 4.0f; // total torque limit [N.m]

    // Tuning: dummy placeholders (0.0f), the real values are withheld.
    static constexpr float K_ENERGY = 0.0f;    // pump gain [N.m per J]
    static constexpr float TAU_SWINGUP = 0.0f; // pump torque budget [N.m]
    static constexpr float K_ARM_P = 0.0f;     // arm-centering stiffness [N.m/rad]
    static constexpr float K_ARM_D = 0.0f;     // arm-centering damping [N.m/(rad/s)]
    static constexpr float E0 = 0.0f;           // target energy, upright [J]

    // Saturated arm torque.
    float compute(const StateVec& x) const
    {
        return clampf(computeRaw(x), TAU_MAX);
    }

    // Torque before the final TAU_MAX clamp (the pump itself is still limited
    // to TAU_SWINGUP).
    float computeRaw(const StateVec& x) const
    {
        const float theta = x(0);
        const float alpha = x(1);
        const float theta_dot = x(2);
        const float alpha_dot = x(3);

        // Pendulum energy about the pivot, zero at the top and
        // -2*m*g*l when hanging down.
        const FurutaParams& p = furuta_params();
        const float jp_pivot = p.Jp + p.mp * p.lc * p.lc; // inertia about the pivot [kg m^2]
        const float mgl = p.mp * p.g * p.lc;              // [J]
        const float cos_a = std::cos(alpha);
        const float E = 0.5f * jp_pivot * alpha_dot * alpha_dot + mgl * (cos_a - 1.0f);
        const float E_err = E - E0;

        // Push in the direction that raises the energy (see header).
        const float dir = -signf(alpha_dot * cos_a);
        float tau = clampf(K_ENERGY * E_err * dir, TAU_SWINGUP);

        // Arm centering, added after the pump clamp so it is not squeezed out
        // of the pump's small budget. The damping term brings the arm in slowly
        // so the hold-up controller inherits little arm velocity.
        tau -= K_ARM_P * theta;
        tau -= K_ARM_D * theta_dot;
        return tau;
    }

  private:
    // Sign with 0 at exactly 0, so the pump is zero at the dead bottom.
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
