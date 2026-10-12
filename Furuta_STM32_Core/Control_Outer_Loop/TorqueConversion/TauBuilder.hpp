// Builds the arm torque that feeds the outer-loop EKF.
//
// The estimator needs the torque applied to the arm, but only the motor
// current is measured. This class smooths the raw current with a low-pass
// filter and converts it to torque (see KeBlend.hpp).
//
// The filter is a causal Butterworth filter. Offline analysis uses a
// zero-phase filter, which cannot run in real time, so the online version
// accepts some phase lag in exchange. The order is kept low to keep that lag
// small.
//
// Called once per outer-loop tick. It reads the latest current published by
// the fast inner loop, which acts as a sample-and-hold onto the 5 ms grid.
//
// For the torque that is actually on the motor right now, use UPrevBuilder;
// this filter is too slow for that.
#pragma once
#include "FurutaModel.hpp"
#include "SampleTimes.hpp"
#include <array>

class TauBuilder
{
  public:
    // fs : rate the filter runs at [Hz], the outer-loop rate
    explicit TauBuilder(float fs = 1.0f / SAMPLE_TIME_OUTER_S)
    {
        design(fs);
    }

    // Returns the arm torque [N.m] for this tick, from the raw current [A].
    float step(float i_raw);

    void reset();

  private:
    static constexpr int ORDER = 2;
    static constexpr float FC_HZ = 25.0f; // filter cutoff

    // One second-order section (transposed direct form II).
    struct Biquad
    {
        float b0, b1, b2, a1, a2;
        float w1 = 0.0f, w2 = 0.0f; // delay states

        float step(float x)
        {
            const float w0 = x - a1 * w1 - a2 * w2;
            const float y = b0 * w0 + b1 * w1 + b2 * w2;
            w2 = w1;
            w1 = w0;
            return y;
        }

        void reset() { w1 = w2 = 0.0f; }
    };

    static constexpr int N_SECTIONS = ORDER / 2;
    std::array<Biquad, N_SECTIONS> sections_{};

    void design(float fs);
};
