// EKF correction step.
//
// Pulls the prediction towards the measured angles. The Kalman gain weighs
// the model against the sensors: noisy sensors (large R) move the estimate
// less, an uncertain model (large P) moves it more.
//
//   S = H*P*H' + R                       innovation covariance
//   K = P*H' * S^-1                      Kalman gain
//   x <- x + K * (z - h(x))
//   P <- (I - K*H) * P * (I - K*H)' + K*R*K'      (Joseph form, keeps P symmetric)
//
//   z  measured [theta, alpha], in model convention
//   R  measurement noise covariance
//   h  measurement model, H its Jacobian (Generated/)
//
// x and P are updated in place.
#pragma once
#include "Types.hpp"
#include "FurutaParams.hpp"

void ekf_correct(Vec6& x, Mat6& P, const Vec2& z, const Mat2& R,
                 const FurutaParams& p);
