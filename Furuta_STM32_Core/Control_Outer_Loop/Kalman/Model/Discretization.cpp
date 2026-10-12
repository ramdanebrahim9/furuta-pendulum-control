#include "Discretization.hpp"
#include "StateDynamics.hpp"
#include "StateJacobian.hpp"

void euler_step(const Vec6& x, float tau, float Ts, const FurutaParams& p, Vec6& x_next)
{
    Vec6 dx;
    ekf_f(x, tau, p, dx);
    x_next = x + Ts * dx;
}

void euler_jacobian(const Vec6& x, float tau, float Ts, const FurutaParams& p, Mat6& Fd)
{
    Mat6 F;
    ekf_F(x, tau, p, F);
    Fd = Mat6::Identity() + F * Ts;
}
