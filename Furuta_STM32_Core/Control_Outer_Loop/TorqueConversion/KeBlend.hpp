// Converts motor current to arm torque.
//
// Torque is the current times the motor's torque constant Ke. The constant is
// slightly different for positive and negative current, so Ke is blended
// smoothly between the two values around zero current. A hard switch would
// make the torque jump at every current reversal.
//
//   blend = 0.5 * (1 + tanh(i / eps_i))     0 for negative current, 1 for positive
//   Ke(i) = Ke_pos * blend + Ke_neg * (1 - blend)
//   tau   = Ke(i) * i
//
// This one function is used wherever current becomes torque, so those uses
// cannot drift apart. CurrentRefBuilder is its inverse.
#pragma once
#include "FurutaModel.hpp"
#include <cmath>

inline float ke_blend_current_to_tau(float i)
{
    const FurutaParams& p = furuta_params();
    const float blend = 0.5f * (1.0f + std::tanh(i / p.eps_i));
    const float ke = p.Ke_pos * blend + p.Ke_neg * (1.0f - blend);
    return ke * i;
}
