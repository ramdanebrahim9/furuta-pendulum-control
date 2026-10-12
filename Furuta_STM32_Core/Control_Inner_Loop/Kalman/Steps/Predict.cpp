#include "Predict.hpp"
#include "../Model/MotorModel.hpp"

EKF_PredictResult EKF_predict(const Eigen::Vector2f& x,
                              const Eigen::Matrix2f& P,
                              const Eigen::Matrix2f& Q,
                              float V,
                              float dt)
{
    const Eigen::Matrix2f F_d = Eigen::Matrix2f::Identity() + motor_model::jacobian_F() * dt;

    EKF_PredictResult res;
    res.x_pred = motor_model::euler_step(x, V, dt);
    res.P_pred = F_d * P * F_d.transpose() + Q;
    return res;
}
