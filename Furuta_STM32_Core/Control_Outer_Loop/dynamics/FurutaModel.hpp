// The plant parameters of the Furuta pendulum, in one place.
//
// generated/FurutaParams.hpp defines the parameter struct (generated from the
// symbolic model). This file fills it with the values and provides the one
// accessor everything uses:
//
//   const FurutaParams& p = furuta_params();
//
// Code that only needs the type (a function taking `const FurutaParams&`) can
// include FurutaParams.hpp directly; code that needs the values includes this.
#pragma once

#include "generated/FurutaParams.hpp"

// ── identified values ────────────────────────────────────────────────────
//
// The numbers below are the plant and motor values identified from
// experiments on the physical rig. They are withheld in this public version
// and shown as 0.0f placeholders; only the structure is real.
//
// Returns a const reference to one static instance: no copy and no per-call
// construction, so it is safe to call from an interrupt. The build uses
// -fno-threadsafe-statics, so the function-local static adds no guard code.
inline const FurutaParams& furuta_params()
{
    static const FurutaParams p = []
    {
        FurutaParams v;

        // Arm (identified)
        v.Jr = 0.0f;     // inertia [kg m^2]
        v.Br = 0.0f;     // viscous friction
        v.Fc_r = 0.0f;   // Coulomb friction [N m]
        v.eps_vr = 0.0f; // velocity width of the Coulomb smoothing [rad/s]

        // Pendulum (identified)
        v.Jp = 0.0f;     // inertia about its centre of mass [kg m^2]
        v.mp = 0.0f;     // mass [kg]
        v.lc = 0.0f;     // distance to the centre of mass [m]
        v.Bp = 0.0f;     // viscous friction
        v.Fc_p = 0.0f;   // Coulomb friction [N m]
        v.eps_vp = 0.0f; // velocity width of the Coulomb smoothing [rad/s]

        v.Lr = 0.0f;     // arm length [m]
        v.g = 9.81f;     // gravity [m/s^2]

        // Motor torque constant, one value per current direction (the motor
        // is slightly asymmetric). Used to convert current to torque.
        v.Ke_pos = 0.0f;
        v.Ke_neg = 0.0f;

        // Half-width [A] of the smooth switch between Ke_pos and Ke_neg
        // around zero current. Hand-tuned, not identified: too narrow gives
        // jerky torque at every current reversal, too wide washes out the
        // asymmetry.
        v.eps_i = 0.0f;
        return v;
    }();
    return p;
}
