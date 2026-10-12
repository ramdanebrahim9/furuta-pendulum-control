#include "Correct.hpp"
#include "../Model/MotorModel.hpp"

EKF_CorrectResult EKF_correct(const Eigen::Vector2f& x_pred,
                              const Eigen::Matrix2f& P_pred,
                              float z,
                              float R)
{
    const Eigen::Matrix<float, 1, 2> H = motor_model::jacobian_H();

    const Eigen::Vector2f PHt = P_pred * H.transpose();
    const float S = (H * PHt)(0, 0) + R;
    const Eigen::Vector2f K = PHt / S;

    const float innovation = z - (H * x_pred)(0, 0);
    const Eigen::Matrix2f IKH = Eigen::Matrix2f::Identity() - K * H;

    EKF_CorrectResult res;
    res.x_est = x_pred + K * innovation;
    res.P_est = IKH * P_pred * IKH.transpose() + K * R * K.transpose();
    return res;
}
