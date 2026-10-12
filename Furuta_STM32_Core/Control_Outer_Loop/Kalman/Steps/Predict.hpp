// EKF prediction step.
//
// Moves the estimate forward one sample using the model, and grows the
// covariance by the process noise to reflect the model's uncertainty.
//
//   x  <- x + Ts * f(x, tau)                (Euler step of the model)
//   P  <- Fd * P * Fd' + Q                  (Fd = I + Ts * F)
//
//   x   state estimate (updated in place)
//   P   covariance (updated in place)
//   tau arm torque applied over the step [N.m]
//   Q   discrete process noise covariance
//   Ts  sample time [s]
#pragma once
#include "Types.hpp"
#include "FurutaParams.hpp"

void ekf_predict(Vec6& x, Mat6& P, float tau, const Mat6& Q, float Ts,
                 const FurutaParams& p);
