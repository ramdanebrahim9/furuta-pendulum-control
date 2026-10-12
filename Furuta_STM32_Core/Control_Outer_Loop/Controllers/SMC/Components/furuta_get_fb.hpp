// Splits the Furuta dynamics into a drift term and an input-gain term.
//
//     q_ddot = f + b * tau
//
//   f : accelerations with the motor off (gravity, Coriolis, friction)
//   b : acceleration produced per unit of arm torque (the input channel)
//
// The sliding-mode law needs exactly this form: f is cancelled by the
// equivalent control and b is inverted to set the torque. This is the only
// part of the SMC that knows the Furuta model; the rest is generic.
//
// The full model is
//     M(q) q_ddot = Bin*tau - C(q,qd) - G(q) - D*qd - Fc(qd),   Bin = [1; 0]
// Torque enters linearly and only through Bin, so the split is exact:
//     f = M \ (-C - G - D*qd - Fc)
//     b = M \ Bin
//
//   q  = [theta; alpha]              (alpha = 0 is upright)
//   qd = [theta_dot; alpha_dot]
//   f  = [f_theta; f_alpha]          (arm, pendulum)
//   b  = [b_theta; b_alpha]
#pragma once
#include <Eigen/Dense>
#include "FurutaParams.hpp"
#include "components/furuta_M.hpp"
#include "components/furuta_Cqd.hpp"
#include "components/furuta_G.hpp"
#include "components/furuta_Fc.hpp"

namespace furuta
{

inline void furuta_get_fb(const Eigen::Matrix<float, 2, 1>& q,
                          const Eigen::Matrix<float, 2, 1>& q_dot,
                          const FurutaParams& p,
                          Eigen::Matrix<float, 2, 1>& f,
                          Eigen::Matrix<float, 2, 1>& b)
{
    Eigen::Matrix<float, 2, 2> M;   furuta_M(q, p, M);          // mass matrix
    Eigen::Matrix<float, 2, 1> Cqd; furuta_Cqd(q, q_dot, p, Cqd); // Coriolis and centrifugal
    Eigen::Matrix<float, 2, 1> G;   furuta_G(q, p, G);          // gravity
    Eigen::Matrix<float, 2, 1> Fc;  furuta_Fc(q_dot, p, Fc);    // Coulomb friction
    Eigen::Matrix<float, 2, 1> Dqd(p.Br * q_dot(0), p.Bp * q_dot(1)); // viscous friction
    Eigen::Matrix<float, 2, 1> Bin(1.0f, 0.0f);                 // torque acts on the arm only

    // One factorisation of the 2x2 mass matrix serves both solves.
    Eigen::LDLT<Eigen::Matrix<float, 2, 2>> ldlt(M);
    f = ldlt.solve(-Cqd - G - Dqd - Fc);
    b = ldlt.solve(Bin);
}

} // namespace furuta
