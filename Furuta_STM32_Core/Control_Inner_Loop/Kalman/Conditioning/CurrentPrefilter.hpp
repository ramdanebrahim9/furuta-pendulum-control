// Band-limits the raw current before the EKF sees it.
//
// Two identical first-order IIR stages in cascade (second order overall).
// The filter is causal, so it adds phase lag; the correction step measures
// the filtered current, not the raw one.
#pragma once
#include <cmath>

class CurrentPrefilter
{
  public:
    CurrentPrefilter(float fc_hz, float ts)
        : alpha_(std::exp(-2.0f * static_cast<float>(M_PI) * fc_hz * ts))
    {
    }

    void reset()
    {
        stage1_ = 0.0f;
        stage2_ = 0.0f;
    }

    float step(float x)
    {
        stage1_ = alpha_ * stage1_ + (1.0f - alpha_) * x;
        stage2_ = alpha_ * stage2_ + (1.0f - alpha_) * stage1_;
        return stage2_;
    }

  private:
    float alpha_;
    float stage1_ = 0.0f;
    float stage2_ = 0.0f;
};
