// Fixed-size Eigen types for the 6-state Furuta estimator.
//
//   State        x = [theta, alpha, theta_dot, alpha_dot, d_r, d_p]
//   Measurement  z = [theta, alpha]
#pragma once
#include <Eigen/Dense>

using Vec6 = Eigen::Matrix<float, 6, 1>;
using Mat6 = Eigen::Matrix<float, 6, 6>;
using Vec2 = Eigen::Matrix<float, 2, 1>;
using Mat2 = Eigen::Matrix<float, 2, 2>;
using Mat26 = Eigen::Matrix<float, 2, 6>;
using Mat62 = Eigen::Matrix<float, 6, 2>;
