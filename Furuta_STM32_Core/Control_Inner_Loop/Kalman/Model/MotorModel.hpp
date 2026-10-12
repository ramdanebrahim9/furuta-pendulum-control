// Electrical model of the motor used by the EKF, state x = [i, d].
//
//   di/dt = (V - R*i + d) / L
//   dd/dt = -d / tau_d
//
// There is no back-EMF or speed term. The disturbance d is a voltage-domain
// state that absorbs back-EMF, model error and any other unmodelled effect,
// and relaxes exponentially with time constant tau_d.
#pragma once
#include <Eigen/Dense>

namespace motor_model
{
// Dummy placeholders (0.0f); the real values are withheld.
constexpr float R = 0.0f;          // winding resistance [ohm]
constexpr float L = 0.0f;          // winding inductance [H]
constexpr float TAU_D = 0.1f;      // disturbance time constant [s]

// Continuous dynamics f(x, V).
Eigen::Vector2f ode(const Eigen::Vector2f& x, float V);

// One forward-Euler step: x + dt * f(x, V).
Eigen::Vector2f euler_step(const Eigen::Vector2f& x, float V, float dt);

// State Jacobian df/dx. Constant, because the model is linear.
Eigen::Matrix2f jacobian_F();

// Measurement Jacobian: only the current is measured, so H = [1 0].
Eigen::Matrix<float, 1, 2> jacobian_H();
} // namespace motor_model
