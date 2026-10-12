// Fast-track gate: detects when the current reference has just moved.
//
// The gate compares the reference now with its value `Window` samples ago.
// It opens on a large step or on a sign flip (outside a small deadband), and
// the look-back keeps it open for one window after the edge without a timer.
// It is also forced open for one window at start-up to ride through the
// initial current inrush.
//
// While open, the EKF lowers its measurement noise and follows the sensor
// closely instead of smoothing the step.
#pragma once
#include <cmath>

template <int Window>
class ReferenceGate
{
  public:
    ReferenceGate(float delta_a, float dir_deadband_a) : delta_(delta_a), deadband_(dir_deadband_a)
    {
    }

    void reset()
    {
        primed_ = false;
        startup_active_ = true;
        startup_left_ = Window;
    }

    // Returns true while the reference is changing (or during start-up).
    bool update(float i_ref)
    {
        if (!primed_)
        {
            for (int n = 0; n <= Window; ++n)
                hist_[n] = i_ref;
            primed_ = true;
        }

        for (int n = Window; n > 0; --n) // index 0 is the newest sample
            hist_[n] = hist_[n - 1];
        hist_[0] = i_ref;

        const float now = hist_[0];
        const float old = hist_[Window];

        const bool jump = std::fabs(now - old) > delta_;
        const bool flip = ((now >= 0.0f) != (old >= 0.0f)) &&
                          (std::fabs(now) > deadband_) &&
                          (std::fabs(old) > deadband_);

        bool startup = false;
        if (startup_active_)
        {
            startup = true;
            if (startup_left_ == 0)
                startup_active_ = false;
            else
                --startup_left_;
        }

        return jump || flip || startup;
    }

  private:
    float delta_;
    float deadband_;
    float hist_[Window + 1] = {0};
    bool primed_ = false;
    bool startup_active_ = true;
    int startup_left_ = Window;
};
