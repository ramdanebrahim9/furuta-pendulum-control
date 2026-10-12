// Process noise covariance Q for the state [i, d].
//
// Scheduled on the signed current reference: a large command means the
// model is less trustworthy, so the current-state noise is raised.
#pragma once
#include <Eigen/Dense>

Eigen::Matrix2f EKF_process_noise(float i_ref);
