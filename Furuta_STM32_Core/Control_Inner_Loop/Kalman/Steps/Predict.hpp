// EKF prediction step.
//
//   x_pred = x + dt * f(x, V)         (Euler step of the motor model)
//   F_d    = I + F*dt                 (discretised Jacobian)
//   P_pred = F_d * P * F_d' + Q
#pragma once
#include <Eigen/Dense>

struct EKF_PredictResult
{
    Eigen::Vector2f x_pred;
    Eigen::Matrix2f P_pred;
};

// x, P : previous estimate and covariance
// Q    : process noise covariance
// V    : voltage applied over the interval
// dt   : sample period
EKF_PredictResult EKF_predict(const Eigen::Vector2f& x,
                              const Eigen::Matrix2f& P,
                              const Eigen::Matrix2f& Q,
                              float V,
                              float dt);
