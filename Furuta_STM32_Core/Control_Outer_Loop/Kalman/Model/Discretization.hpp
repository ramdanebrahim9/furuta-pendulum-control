// Converts the continuous Furuta model into discrete-time form with a single
// forward-Euler step, which is accurate enough at the 5 ms sample time.
//
//   state     x_next = x + Ts * f(x, tau)
//   Jacobian  Fd     = I + Ts * F(x, tau)
//
// f and F are the generated continuous model and its Jacobian (Generated/).
#pragma once
#include "Types.hpp"
#include "FurutaParams.hpp"

// Propagates the state by one sample.
void euler_step(const Vec6& x, float tau, float Ts, const FurutaParams& p, Vec6& x_next);

// Discrete-time Jacobian used to propagate the covariance.
void euler_jacobian(const Vec6& x, float tau, float Ts, const FurutaParams& p, Mat6& Fd);
