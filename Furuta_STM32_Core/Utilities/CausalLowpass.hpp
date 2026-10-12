// First-order low-pass filter that can be switched off.
//
// A Butterworth filter from the bilinear transform of an analog prototype,
// implemented in transposed direct form II (one delay):
//
//   w[k] = x[k] + a1 * w[k-1]
//   y[k] = b0 * (w[k] + w[k-1])
//
// The filter is causal, so it adds phase lag; that is the price of running
// online. When `enabled` is false, step() returns its input unchanged and no
// state is kept, so a disabled filter has no start-up behaviour to account for.
//
// One filter instance smooths one signal (arm speed, pendulum speed, the
// torque command, the measured current). It defaults to the outer-loop rate;
// the inner loop passes its own 10 kHz rate.
#pragma once
#include <cmath>
#include "SampleTimes.hpp"

class CausalLowpass
{
  public:
    // fc_hz   : cutoff frequency [Hz]
    // enabled : false makes step() a passthrough
    // fs_hz   : rate at which step() is called [Hz]
    CausalLowpass(float fc_hz, bool enabled, float fs_hz = 1.0f / SAMPLE_TIME_OUTER_S)
        : enabled_(enabled)
    {
        if (enabled_)
        {
            const float w0 = 2.0f * static_cast<float>(M_PI) * fc_hz / fs_hz;
            const float wd = std::tan(w0 / 2.0f); // pre-warped analog cutoff
            b0_ = wd / (1.0f + wd);
            a1_ = (1.0f - wd) / (1.0f + wd);
        }
    }

    float step(float x_raw)
    {
        if (!enabled_)
            return x_raw;

        const float w = x_raw + a1_ * w_state_;
        const float y = b0_ * (w + w_state_);
        w_state_ = w;
        return y;
    }

    void reset() { w_state_ = 0.0f; }

  private:
    bool enabled_;
    float b0_ = 0.0f;
    float a1_ = 0.0f;
    float w_state_ = 0.0f;
};
