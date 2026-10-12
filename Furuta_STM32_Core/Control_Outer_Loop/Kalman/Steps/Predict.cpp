#include "Predict.hpp"
#include "Discretization.hpp"

void ekf_predict(Vec6& x, Mat6& P, float tau, const Mat6& Q, float Ts,
                 const FurutaParams& p)
{
    Vec6 x_pred;
    Mat6 Fd;
    euler_step(x, tau, Ts, p, x_pred);
    euler_jacobian(x, tau, Ts, p, Fd);

    P = Fd * P * Fd.transpose() + Q;
    x = x_pred;
}
