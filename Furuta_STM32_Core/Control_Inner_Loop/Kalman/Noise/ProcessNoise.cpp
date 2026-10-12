#include "ProcessNoise.hpp"

Eigen::Matrix2f EKF_process_noise(float i_ref)
{
    // Dummy placeholders (0.0f); the real schedule values are withheld.
    // Standard deviations of the process noise.
    static constexpr float Q_I_NOMINAL = 0.0f;  // moderate command
    static constexpr float Q_I_POS_HARD = 0.0f; // i_ref above POS_THRESH_A
    static constexpr float Q_I_NEG_HARD = 0.0f; // i_ref below NEG_THRESH_A
    static constexpr float Q_D = 0.0f;          // disturbance
    static constexpr float POS_THRESH_A = 0.0f;
    static constexpr float NEG_THRESH_A = 0.0f;

    float q_i = Q_I_NOMINAL;
    if (i_ref > POS_THRESH_A)
        q_i = Q_I_POS_HARD;
    else if (i_ref < NEG_THRESH_A)
        q_i = Q_I_NEG_HARD;

    Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
    Q(0, 0) = q_i * q_i;
    Q(1, 1) = Q_D * Q_D;
    return Q;
}
