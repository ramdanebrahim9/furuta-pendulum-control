// Measures the arm torque currently on the motor.
//
// The velocity-form MPCs add an increment to the previous torque, so they
// need to know that torque. Using their own last output would ignore
// everything between the command and the motor (missed Pi replies, saturation,
// a current loop that is not tracking). This class measures it from the motor
// current instead.
//
// It is separate from TauBuilder, which is smoothed for the EKF and lags
// about a tick. The ripple filtering here runs in the 10 kHz inner loop:
// sampling the raw current every 5 ms would alias the PWM ripple into the
// sample, and no filter at 200 Hz can remove it afterwards. That filter adds
// only about 0.4 ms of lag, so a current step shows up almost in full at the
// next outer tick.
//
// Per outer tick, step() reads the filtered current and converts it to torque
// with the shared blend (KeBlend.hpp).
#pragma once
#include "KeBlend.hpp"
#include "Sensors.hpp"

class UPrevBuilder
{
  public:
    // Call once per outer-loop tick, before any controller runs.
    // Returns the measured torque [N.m].
    float step()
    {
        u_prev_ = ke_blend_current_to_tau(sensors_read_current_filt());
        return u_prev_;
    }

    float u_prev() const { return u_prev_; }

    void reset() { u_prev_ = 0.0f; }

  private:
    float u_prev_ = 0.0f;
};
