#include "EKF.hpp"

EKF_State EKF::state_;
CurrentPrefilter EKF::prefilter_(1200.0f, EKF::TS);
ReferenceGate<EKF::WINDOW> EKF::gate_(0.08f, 0.02f); // step size [A], sign-flip deadband [A]
RollingStd<EKF::WINDOW> EKF::std_;

void EKF::init()
{
    // Start from zero current and disturbance with moderate uncertainty.
    state_.x_est.setZero();
    state_.P_est = Eigen::Matrix2f::Zero();
    state_.P_est(0, 0) = 0.1f * 0.1f;   // i
    state_.P_est(1, 1) = 0.08f * 0.08f; // d

    prefilter_.reset();
    gate_.reset();
    std_.reset();
}

const EKF_State& EKF::step(float u, float i_meas, float i_ref)
{
    const float i_filt = prefilter_.step(i_meas);
    const bool fast_track = gate_.update(i_ref);
    const float meas_std = std_.update(i_filt);

    const Eigen::Matrix2f Q = EKF_process_noise(i_ref);
    const float R = EKF_measurement_noise(fast_track, meas_std);

    const EKF_PredictResult pred = EKF_predict(state_.x_est, state_.P_est, Q, u, TS);
    const EKF_CorrectResult corr = EKF_correct(pred.x_pred, pred.P_pred, i_filt, R);

    state_.x_est = corr.x_est;
    state_.P_est = corr.P_est;
    return state_;
}
