#include "MeasurementNoise.hpp"
#include <cmath>

float EKF_measurement_noise(bool fast_track, float meas_std)
{
    // Dummy placeholders (0.0f); the real schedule values are withheld.
    static constexpr float R_STD_FAST_TRACK = 0.0f; // std during fast-track [A]
    static constexpr float STD_MIN = 0.0f;          // clamp on the measured std [A]
    static constexpr float STD_MAX = 0.0f;
    static constexpr float STD_INFLATE = 0.0f;       // trust the sensor less in steady state

    float r_std = R_STD_FAST_TRACK;
    if (!fast_track)
        r_std = STD_INFLATE * fmaxf(STD_MIN, fminf(STD_MAX, meas_std));

    return r_std * r_std;
}
