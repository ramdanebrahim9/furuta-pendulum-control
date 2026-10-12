#include "FurutaEKF.hpp"

using namespace furuta_ekf_tuning;

FurutaEKF_State FurutaEKF::state_;
Mat6 FurutaEKF::Q_;
Mat2 FurutaEKF::R_;

void FurutaEKF::init()
{
    // Zero everywhere except alpha, which starts hanging down.
    state_.x_est.setZero();
    state_.x_est(1) = FURUTA_ALPHA_OFFSET;

    state_.P_est.setZero();
    state_.P_est(0, 0) = P0_POS;
    state_.P_est(1, 1) = P0_POS;
    state_.P_est(2, 2) = P0_VEL;
    state_.P_est(3, 3) = P0_VEL;
    state_.P_est(4, 4) = P0_DST;
    state_.P_est(5, 5) = P0_DST;

    // Process noise, scaled by the sample time to get per-step values.
    Q_.setZero();
    Q_(0, 0) = Q_POS * SAMPLE_TIME_OUTER_S;
    Q_(1, 1) = Q_POS * SAMPLE_TIME_OUTER_S;
    Q_(2, 2) = Q_VEL * SAMPLE_TIME_OUTER_S;
    Q_(3, 3) = Q_VEL * SAMPLE_TIME_OUTER_S;
    Q_(4, 4) = Q_DST_R * SAMPLE_TIME_OUTER_S;
    Q_(5, 5) = Q_DST_P * SAMPLE_TIME_OUTER_S;

    // Measurement noise: encoder quantisation, same for both angles.
    R_.setZero();
    R_(0, 0) = ENC_RES * ENC_RES * R_SCALE;
    R_(1, 1) = ENC_RES * ENC_RES * R_SCALE;
}

const FurutaEKF_State& FurutaEKF::step(float tau, float theta_enc, float pendl_enc)
{
    const FurutaParams& p = furuta_params();

    // The arm encoder and the model share the same zero; the pendulum
    // encoder is converted to the model convention.
    Vec2 z;
    z(0) = theta_enc;
    z(1) = pendl_enc_to_model(pendl_enc);

    ekf_predict(state_.x_est, state_.P_est, tau, Q_, SAMPLE_TIME_OUTER_S, p);
    ekf_correct(state_.x_est, state_.P_est, z, R_, p);

    return state_;
}
