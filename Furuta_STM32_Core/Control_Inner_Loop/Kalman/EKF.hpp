// Extended Kalman filter for the motor current loop.
//
// State        x = [i, d]: winding current and a lumped voltage disturbance
//              (back-EMF, model error, load) that relaxes slowly.
// Measurement  the current only.
//
// Each step() call runs, in order:
//   1. CurrentPrefilter  band-limit the raw current (1.2 kHz)
//   2. ReferenceGate     detect reference edges (fast-track)
//      RollingStd        local noise level of the filtered current
//   3. Noise/            adaptive Q (from i_ref) and R (from gate and std)
//   4. Steps/Predict     propagate the state with the applied voltage
//   5. Steps/Correct     update with the filtered current
//
// Adaptive R is the key idea: around reference edges R is small so the
// filter follows real current steps, and in steady state R follows the
// measured ripple so the estimate stays smooth.
#pragma once
#include <Eigen/Dense>
#include "Conditioning/CurrentPrefilter.hpp"
#include "Conditioning/ReferenceGate.hpp"
#include "Conditioning/RollingStd.hpp"
#include "Noise/MeasurementNoise.hpp"
#include "Noise/ProcessNoise.hpp"
#include "SampleTimes.hpp"
#include "Steps/Correct.hpp"
#include "Steps/Predict.hpp"

struct EKF_State
{
    Eigen::Vector2f x_est; // [i, d]
    Eigen::Matrix2f P_est;
};

class EKF
{
  public:
    static constexpr float TS = SAMPLE_TIME_INNER_S; // 100 us

    // Gate and std window: 2.2 ms of history.
    static constexpr int WINDOW = static_cast<int>(2.2e-3f / TS);

    static void init();

    // One filter step per control tick.
    //   u      : voltage applied over the last interval
    //   i_meas : raw measured current
    //   i_ref  : current reference (drives the gate and Q)
    static const EKF_State& step(float u, float i_meas, float i_ref);

  private:
    static EKF_State state_;
    static CurrentPrefilter prefilter_;
    static ReferenceGate<WINDOW> gate_;
    static RollingStd<WINDOW> std_;
};
