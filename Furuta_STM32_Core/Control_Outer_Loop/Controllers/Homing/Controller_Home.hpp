// Homing controller: brings the arm back to theta = 0 after a fall.
//
// Active in the HOMING state, which the state machine enters when the
// pendulum leaves the LQR catch window. The pendulum is left to hang; only
// the arm is controlled. This puts the arm at a known angle before the next
// swing-up starts, so the cable never winds up against the mechanical limit.
//
// Control law, a PD on the arm angle and rate:
//
//   tau = sat( -K_P * theta - K_D * theta_dot,  +-TAU_MAX )
//
// State  x = [theta, alpha, theta_dot, alpha_dot]
// Output tau = arm torque [N.m]
//
// The interface (StateVec, compute) matches the other controllers, so the
// outer loop can call any of them the same way.
#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace furuta
{

class Controller_Home
{
  public:
    static constexpr int NX = 4;
    using StateVec = Eigen::Matrix<float, NX, 1>;

    static constexpr float TAU_MAX = 0.08f; // torque limit [N.m]

    // Gains: dummy placeholders (0.0f), the real values are withheld.
    static constexpr float K_P = 0.0f; // centering stiffness [N.m/rad]
    static constexpr float K_D = 0.0f; // centering damping [N.m/(rad/s)]

    // Saturated arm torque.
    float compute(const StateVec& x) const
    {
        const float tau = computeRaw(x);
        return std::fmax(-TAU_MAX, std::fmin(TAU_MAX, tau));
    }

    // Torque before saturation.
    float computeRaw(const StateVec& x) const
    {
        return -K_P * x(0) - K_D * x(2);
    }

    // Homing is finished once the arm is within 3 degrees of center.
    // The state machine uses this to move from HOMING to SWING_UP.
    //
    // Position only, on purpose. Swing-up needs a centered arm, not a
    // stationary one, and a velocity condition could leave the state machine
    // stuck in HOMING: with a soft PD the arm rate hovers near the threshold
    // instead of cleanly crossing it.
    static constexpr float THETA_SETTLED_RAD = 3.0f * static_cast<float>(M_PI) / 180.0f;

    static bool isSettled(const StateVec& x)
    {
        return std::fabs(x(0)) <= THETA_SETTLED_RAD;
    }
};

} // namespace furuta
