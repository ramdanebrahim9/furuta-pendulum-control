#include "Correct.hpp"
#include "MeasurementModel.hpp"
#include "MeasurementJacobian.hpp"

void ekf_correct(Vec6& x, Mat6& P, const Vec2& z, const Mat2& R,
                 const FurutaParams& p)
{
    Mat26 H;
    ekf_H(x, p, H);

    const Mat62 PHt = P * H.transpose();
    const Mat2 S = H * PHt + R;
    const Mat62 K = PHt * S.inverse(); // 2x2 inverse, cheap

    Vec2 h;
    ekf_h(x, p, h);
    const Vec2 residual = z - h;

    x = x + K * residual;

    const Mat6 IKH = Mat6::Identity() - K * H;
    P = IKH * P * IKH.transpose() + K * R * K.transpose();
}
