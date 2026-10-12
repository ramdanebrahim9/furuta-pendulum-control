#include "TauBuilder.hpp"
#include "KeBlend.hpp"
#include <cmath>

// Builds the Butterworth low-pass as a cascade of second-order sections,
// computed from ORDER, FC_HZ and fs rather than stored as a coefficient table,
// so it stays correct if any of them change.
//
//   1. The Butterworth poles lie evenly spaced on the left half of the unit
//      circle in the analog domain.
//   2. Each complex pole pair makes one analog second-order section.
//   3. The bilinear transform turns each section into a digital biquad.
void TauBuilder::design(float fs)
{
    const float wd = std::tan(static_cast<float>(M_PI) * FC_HZ / fs); // pre-warped cutoff

    for (int k = 0; k < N_SECTIONS; ++k)
    {
        // Angle of this pole pair.
        const float angle = static_cast<float>(M_PI) / 2.0f
                            + static_cast<float>(M_PI) * (2.0f * k + 1.0f) / (2.0f * ORDER);
        const float c = std::cos(angle);

        // Analog section wd^2 / (s^2 - 2*c*wd*s + wd^2) after the bilinear transform.
        const float wd2 = wd * wd;
        const float norm = 1.0f / (1.0f - 2.0f * c * wd + wd2);

        Biquad& sec = sections_[k];
        sec.b0 = wd2 * norm;
        sec.b1 = 2.0f * sec.b0;
        sec.b2 = sec.b0;
        sec.a1 = 2.0f * (wd2 - 1.0f) * norm;
        sec.a2 = (1.0f + 2.0f * c * wd + wd2) * norm;
        sec.reset();
    }
}

void TauBuilder::reset()
{
    for (auto& sec : sections_)
        sec.reset();
}

float TauBuilder::step(float i_raw)
{
    float i = i_raw;
    for (auto& sec : sections_)
        i = sec.step(i);

    return ke_blend_current_to_tau(i);
}
