// Neural-network hold-up controller (balances the pendulum upright).
//
// A small MLP (7 -> 64 tanh -> 64 tanh -> 1) trained by behaviour cloning:
// it imitates a nonlinear MPC (solved offline with acados) so the STM32 can
// reproduce its behaviour with a fixed, cheap forward pass instead of an
// online optimisation. It runs entirely on the STM32; no Pi is involved.
//
//   State  x = [theta, alpha, theta_dot, alpha_dot]   (alpha = 0 is upright)
//   Output u = arm torque [N.m], saturated to +-TAU_MAX
//
// Network input (the order is fixed by training):
//   [sin(theta), cos(theta), sin(alpha), cos(alpha), theta_dot, alpha_dot, u_prev]
//
// Angles enter as sin/cos so the network sees them as continuous across the
// wrap-around. u_prev is the torque actually measured on the motor: the MPC
// being imitated is in increment (velocity) form, so the same state can call
// for different torques depending on the previous one.
//
// Input normalisation and output scaling are folded into the weights, so raw
// features are passed in.
//
// The forward pass is MATLAB Coder output (Generated/nn_stabilize_predict.c).
// Retraining only means replacing the files in Generated/.
//
// The network was trained on data sampled every TS; call compute() once per TS.
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "nn_stabilize_predict.h"

namespace furuta {

class Controller_NN_HoldUp
{
  public:
    static constexpr int NX = 4;
    using StateVec = Eigen::Matrix<float, NX, 1>;

    static constexpr float TAU_MAX = 5.0f; // torque limit [N.m]
    static constexpr float TS = 0.005f;    // sample time of the training data [s]

    // Saturated arm torque.
    // x      : state
    // u_prev : torque measured on the motor this tick [N.m]
    float compute(const StateVec& x, float u_prev) const
    {
        const float u = computeRaw(x, u_prev);
        return u > TAU_MAX ? TAU_MAX : (u < -TAU_MAX ? -TAU_MAX : u);
    }

    // Network output before saturation.
    float computeRaw(const StateVec& x, float u_prev) const
    {
        const float features[7] = {
            std::sin(x(0)), std::cos(x(0)),
            std::sin(x(1)), std::cos(x(1)),
            x(2), x(3),
            u_prev,
        };
        return nn_stabilize_predict(features);
    }
};

} // namespace furuta
