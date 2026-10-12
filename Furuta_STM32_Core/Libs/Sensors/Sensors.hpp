// The data bus between the fast inner loop and the slower outer loop.
//
//   inner loop (100 us)                          outer loop (5 ms)
//   -------------------                          -----------------
//   sensors_sample()   encoders  -- seqlock -->  sensors_read_angles()
//   publish_current()  motor current ---------->  read_current()
//   publish_current_filt() smoothed current -->  read_current_filt()
//   read_tau()         <------------------------ publish_tau()  torque demand
//
// Sampling lives here, in a layer of its own, because the encoders belong to
// neither loop. The estimator and balance controllers (outer) are their main
// users, but the arm angle must be sampled at the fast rate to protect the
// arm and cable. A separate layer lets the sampling rate and the ownership of
// the angles be decided independently.
//
// Rules
//   - sensors_sample() is the only caller of Encoder::update(). It runs from
//     the fast tick, before either loop steps, so both see the same sample.
//   - Everything else only reads. Single values (current, torque) are plain
//     floats: aligned 32-bit accesses are atomic on the Cortex-M7, so a
//     reader never sees a half-written value.
//   - A coherent pair (arm and pendulum angle) uses a seqlock, so the two
//     always come from the same sample.
#pragma once
#include <cstdint>

#ifdef __cplusplus
extern "C"
{
#endif

    // Samples all fast-rate sensors. Call from the fast tick, before the
    // inner loop steps, from one place only.
    void sensors_sample(void);

    // Zeros the encoder positions. Call at the start of every run.
    void sensors_reset(void);

#ifdef __cplusplus
}

// Arm and pendulum angle from the same sample, load side.
struct SensorAngles
{
    float arm_rad;
    float pendl_rad;
};

// Retries if a new sample lands mid-read, so the pair never mixes two samples.
// Lock-free, and bounded in practice since the writer is a few instructions at
// a fixed rate.
SensorAngles sensors_read_angles(void);

// Arm angle alone [deg], used by the arm safety check.
float sensors_arm_angle_deg(void);

// Inner loop -> outer loop: the measured motor current [A].
void sensors_publish_current(float i_meas);
float sensors_read_current(void);

// Inner loop -> outer loop: the current after a light low-pass at the fast
// rate. The outer loop uses it to measure the torque on the motor; it must be
// filtered here, because sampling the raw current every 5 ms would alias the
// PWM ripple into the result.
void sensors_publish_current_filt(float i_filt);
float sensors_read_current_filt(void);

// Outer loop -> inner loop: the demanded arm torque [N.m]. The inner loop
// converts it to a current reference.
void sensors_publish_tau(float tau_cmd);
float sensors_read_tau(void);

#endif
