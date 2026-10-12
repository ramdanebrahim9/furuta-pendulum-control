// EKF correction step with a scalar current measurement.
//
// With H = [1 0] and scalar R, the innovation covariance S is a scalar and
// the Kalman gain K is a 2x1 column.
//
//   S     = H*P*H' + R
//   K     = P*H' / S
//   x_est = x_pred + K * (z - H*x_pred)
//   P_est = (I - K*H) * P * (I - K*H)' + K*R*K'      (Joseph form)
#pragma once
#include <Eigen/Dense>

struct EKF_CorrectResult
{
    Eigen::Vector2f x_est;
    Eigen::Matrix2f P_est;
};

// z : measured (filtered) current, R : measurement noise variance
EKF_CorrectResult EKF_correct(const Eigen::Vector2f& x_pred,
                              const Eigen::Matrix2f& P_pred,
                              float z,
                              float R);
