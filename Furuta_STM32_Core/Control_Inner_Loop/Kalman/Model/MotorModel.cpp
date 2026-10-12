#include "MotorModel.hpp"

namespace motor_model
{
Eigen::Vector2f ode(const Eigen::Vector2f& x, float V)
{
    const float i = x(0);
    const float d = x(1);

    Eigen::Vector2f dx;
    dx(0) = (V - R * i + d) / L;
    dx(1) = -d / TAU_D;
    return dx;
}

Eigen::Vector2f euler_step(const Eigen::Vector2f& x, float V, float dt)
{
    return x + dt * ode(x, V);
}

Eigen::Matrix2f jacobian_F()
{
    Eigen::Matrix2f F;
    F << -R / L, 1.0f / L,
          0.0f, -1.0f / TAU_D;
    return F;
}

Eigen::Matrix<float, 1, 2> jacobian_H()
{
    Eigen::Matrix<float, 1, 2> H;
    H << 1.0f, 0.0f;
    return H;
}
} // namespace motor_model
