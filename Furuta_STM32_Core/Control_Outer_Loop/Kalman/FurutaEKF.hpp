// Extended Kalman filter for the Furuta pendulum (outer loop, every 5 ms).
//
// Estimates the full mechanical state from two encoders and the applied
// arm torque.
//
//   State        x = [theta, alpha, theta_dot, alpha_dot, d_r, d_p]
//   Measurement  z = [theta, alpha]        (the two encoder angles)
//   Input        tau = arm torque [N.m]
//
//   theta, alpha          arm and pendulum angles (alpha = pi hanging down)
//   theta_dot, alpha_dot  their rates. Neither is measured; both are
//                         estimated from the model and the encoder angles.
//   d_r, d_p              disturbance torques on the arm and pendulum.
//                         They absorb friction and everything the model
//                         leaves out, so no Coulomb friction term is needed.
//
// Each step runs predict, then correct:
//   Steps/Predict   advance the model one sample using the applied torque
//   Steps/Correct   pull the result towards the measured angles
// The model and its Jacobians are generated from the symbolic equations of
// motion (Generated/); Model/ discretises them.
//
// Angle convention: the pendulum encoder reads 0 when hanging down, while
// the model uses alpha = pi for down. step() applies the offset (see
// FurutaConventions.hpp). A constant offset disappears when differentiated,
// so the rates need no correction.
//
// This class is the estimator only. Smoothing of the estimated rates
// happens in the outer loop.
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "FurutaConventions.hpp"
#include "FurutaModel.hpp"
#include "SampleTimes.hpp"
#include "Steps/Correct.hpp"
#include "Steps/Predict.hpp"
#include "Types.hpp"

// Noise and covariance settings. Dummy placeholders (0.0f); the real tuning
// values are withheld.
namespace furuta_ekf_tuning
{
// Measurement noise: encoder quantisation. 500 PPR with x4 quadrature
// decoding gives 2000 counts per revolution.
static constexpr float ENC_PPR = 500.0f;
static constexpr float ENC_QUAD = 4.0f;
static constexpr float ENC_RES = 2.0f * static_cast<float>(M_PI) / (ENC_PPR * ENC_QUAD); // [rad]
static constexpr float R_SCALE = 0.0f; // R = ENC_RES^2 * R_SCALE

// Process noise per second
static constexpr float Q_POS = 0.0f;   // angles
static constexpr float Q_VEL = 0.0f;   // rates
static constexpr float Q_DST_R = 0.0f; // arm disturbance (fast: follows Coulomb friction)
static constexpr float Q_DST_P = 0.0f; // pendulum disturbance (slow)

// Initial covariance
static constexpr float P0_POS = 0.0f;
static constexpr float P0_VEL = 0.0f;
static constexpr float P0_DST = 0.0f;
} // namespace furuta_ekf_tuning

struct FurutaEKF_State
{
    Vec6 x_est; // [theta, alpha, theta_dot, alpha_dot, d_r, d_p]
    Mat6 P_est;
};

class FurutaEKF
{
  public:
    // Starts from zero with the pendulum hanging down (alpha = pi).
    static void init();

    // One filter step per outer-loop tick.
    //   tau       : arm torque over this step [N.m]
    //   theta_enc : arm encoder angle [rad]
    //   pendl_enc : pendulum encoder angle [rad], encoder convention
    static const FurutaEKF_State& step(float tau, float theta_enc, float pendl_enc);

  private:
    static FurutaEKF_State state_;
    static Mat6 Q_; // process noise, built in init()
    static Mat2 R_; // measurement noise, built in init()
};
