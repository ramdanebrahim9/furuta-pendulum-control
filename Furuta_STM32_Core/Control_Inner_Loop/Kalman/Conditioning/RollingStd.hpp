// Causal rolling sample standard deviation (n-1) over the last `Window`
// samples. Uses a partial window until the buffer fills, and returns 0 until
// two samples are available.
//
// The EKF uses it as a local noise estimate: R grows when the current is
// ripply and shrinks when it is quiet.
#pragma once
#include <cmath>

template <int Window>
class RollingStd
{
  public:
    void reset()
    {
        idx_ = 0;
        count_ = 0;
    }

    float update(float x)
    {
        buf_[idx_] = x;
        idx_ = (idx_ + 1) % Window;
        if (count_ < Window)
            ++count_;

        if (count_ < 2)
            return 0.0f;

        float mean = 0.0f;
        for (int n = 0; n < count_; ++n)
            mean += buf_[n];
        mean /= static_cast<float>(count_);

        float acc = 0.0f;
        for (int n = 0; n < count_; ++n)
        {
            const float e = buf_[n] - mean;
            acc += e * e;
        }
        return std::sqrt(acc / static_cast<float>(count_ - 1));
    }

  private:
    float buf_[Window] = {0};
    int idx_ = 0;
    int count_ = 0;
};
