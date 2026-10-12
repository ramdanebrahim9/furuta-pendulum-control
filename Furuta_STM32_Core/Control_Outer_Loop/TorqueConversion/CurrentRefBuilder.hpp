// Converts a demanded torque into a current reference for the inner loop.
//
// The inverse of KeBlend. The outer loop's controllers ask for a torque; the
// motor current controller works in amps and knows nothing about torque
// constants. This class sits between them.
//
// Forward (KeBlend):  tau = Ke(i) * i, where Ke depends on the sign of i.
//
// Inverse: Ke depends on the very current being solved for. But Ke_pos and
// Ke_neg are both positive and similar in size, so tau always has the same
// sign as i. The blend can therefore be driven by the torque directly:
//
//   blend = 0.5 * (1 + tanh(tau / eps_tau))
//   Ke    = Ke_pos * blend + Ke_neg * (1 - blend)
//   i_ref = tau / Ke
//
// eps_tau is the current-axis blend width eps_i converted to torque units,
// eps_i * average(Ke_pos, Ke_neg), so the smoothing covers the same physical
// range as the forward blend.
#pragma once
#include "FurutaModel.hpp"
#include <cmath>

class CurrentRefBuilder
{
  public:
    // Called every tick with the demanded torque [N.m]. Returns the current
    // reference [A] for the current controller.
    static float step(float tau_ref)
    {
        const FurutaParams& p = furuta_params();

        const float ke_avg = 0.5f * (p.Ke_pos + p.Ke_neg);
        const float eps_tau = p.eps_i * ke_avg;

        const float blend = 0.5f * (1.0f + std::tanh(tau_ref / eps_tau));
        const float ke = p.Ke_pos * blend + p.Ke_neg * (1.0f - blend);

        return tau_ref / ke;
    }
};
