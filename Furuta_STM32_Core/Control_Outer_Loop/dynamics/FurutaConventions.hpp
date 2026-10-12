// Angle conventions shared by the estimator and the controllers.
//
// The pendulum encoder and the model disagree on where zero is:
//
//   encoder : pendl_enc = 0  is hanging down
//   model   : alpha     = pi is hanging down (alpha = 0 is upright)
//
// pendl_enc_to_model() is the one place that converts between them, so the
// offset is never re-added by hand elsewhere. The arm needs no conversion:
// its encoder and the model share the same zero.
//
// The estimator keeps alpha continuous (it counts past a full turn) on
// purpose. A wrapped angle jumps at +-pi, and an estimator propagating
// through that jump would see a huge false velocity. wrap_pi() is for code
// outside the estimator that wants a bounded angle, such as a controller that
// must treat "five turns past upright" the same as "upright".
#pragma once
#include <cmath>

static constexpr float FURUTA_ALPHA_OFFSET = 3.14159265358979323846f; // pi
static constexpr float FURUTA_TWO_PI       = 2.0f * FURUTA_ALPHA_OFFSET;

inline float pendl_enc_to_model(float pendl_enc_rad)
{
    return pendl_enc_rad + FURUTA_ALPHA_OFFSET;
}

// Folds any angle [rad] into [-pi, pi]. Not for the estimator's own state;
// see the note above.
inline float wrap_pi(float angle_rad)
{
    float wrapped = std::fmod(angle_rad + FURUTA_ALPHA_OFFSET, FURUTA_TWO_PI);
    if (wrapped < 0.0f)
        wrapped += FURUTA_TWO_PI;
    return wrapped - FURUTA_ALPHA_OFFSET;
}
