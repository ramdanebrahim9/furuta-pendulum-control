// Reads a quadrature encoder through a hardware timer.
//
// The timer counts encoder edges in a 16-bit register that wraps around. This
// class unwraps it into a continuous signed position, so the arm angle can
// keep growing past a full turn, and converts counts to an angle on the load
// side of the gearbox. The implementation is hardware specific and is not part
// of this showcase.
//
// Threading
//   update() has exactly one caller, sensors_sample(), running in the fast
//   control interrupt. It is not reentrant: it updates the position and the
//   previous count together, and a second caller preempting it would add the
//   same movement twice and corrupt the position permanently.
//
//   Everyone else reads through the snapshot functions, which are safe from
//   any interrupt priority. An aligned 32-bit load is atomic on the Cortex-M7,
//   so a reader never sees a half-written position.
//
//   A reader that needs two encoders from the same instant (arm and pendulum)
//   uses seq() as a seqlock: read seq, read the angles, read seq again, and
//   retry if it changed. See sensors_read_angles().
#pragma once
#include <stdint.h>

struct __TIM_HandleTypeDef;
typedef struct __TIM_HandleTypeDef TIM_HandleTypeDef;

class Encoder
{
  public:
    // htim      : timer configured in encoder mode
    // ppr       : pulses per revolution of the encoder (4x decoding is applied)
    // gearRatio : encoder turns per turn of the load (1 = direct)
    Encoder(TIM_HandleTypeDef* htim, uint32_t ppr, float gearRatio = 1.0f);

    // Starts the timer in encoder mode.
    bool start();

    // Resets the position to zero.
    void clear();

    // Reads the hardware counter and adds the movement since the last call.
    // Single caller only, see the threading note above.
    void update();

    // Load-side angle from the latest update, safe to call from any priority.
    float snapshotOutputAngleRad() const;
    float snapshotOutputAngleDeg() const;

    // Increases on every update(); used as a seqlock.
    uint32_t seq() const;
};
