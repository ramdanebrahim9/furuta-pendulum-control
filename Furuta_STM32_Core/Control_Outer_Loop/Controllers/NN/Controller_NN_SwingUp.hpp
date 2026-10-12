// Neural-network swing-up controller.
//
// Sibling of Controller_NN_HoldUp. A small MLP (5 -> 64 tanh -> 64 tanh -> 1)
// trained by behaviour cloning of a swing-up nonlinear MPC. It pumps energy
// into the pendulum from the hanging position until it reaches the region
// where the hold-up controller can take over. Runs entirely on the STM32.
//
//   State  x = [theta, alpha, theta_dot, alpha_dot]   (alpha = 0 is upright)
//   Output u = arm torque [N.m], saturated to +-TAU_MAX
//
// Network input (the order is fixed by training):
//   [theta, sin(alpha), cos(alpha), theta_dot, alpha_dot]
//
// Differences from the hold-up network:
//   - theta is used directly. The arm angle is bounded by the cable, so it
//     needs no wrap-around handling; only alpha does.
//   - there is no u_prev input. The swing-up MPC works on the torque
//     directly, not on increments, so the previous torque adds nothing.
//
// Input normalisation and output scaling are folded into the weights, so raw
// features are passed in.
//
// The forward pass is MATLAB Coder output (Generated/nn_swingup_predict.c).
// Retraining only means replacing the files in Generated/.
//
// The network was trained on data sampled every TS; call compute() once per TS.
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "nn_swingup_predict.h"

namespace furuta
{

class Controller_NN_SwingUp
{
  public:
    static constexpr int NX = 4;
    using StateVec = Eigen::Matrix<float, NX, 1>;

    static constexpr float TAU_MAX = 0.8f; // torque limit [N.m], lower than the hold-up limit
    static constexpr float TS = 0.005f;    // sample time of the training data [s]

    // Saturated arm torque.
    float compute(const StateVec& x) const
    {
        const float u = computeRaw(x);
        return u > TAU_MAX ? TAU_MAX : (u < -TAU_MAX ? -TAU_MAX : u);
    }

    // Network output before saturation.
    float computeRaw(const StateVec& x) const
    {
        const float features[5] = {
            x(0),
            std::sin(x(1)),
            std::cos(x(1)),
            x(2),
            x(3),
        };
        return nn_swingup_predict(features);
    }
};

} // namespace furuta
