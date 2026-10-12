#pragma once

#include <cmath>

// Discrete PI current controller with back-calculation anti-windup.
//
//   e      = ref - meas
//   u_raw  = Kp*e + Ki*Ts*intg
//   u_sat  = clamp(u_raw, -V_max, +V_max)
//   intg  += (not saturated ? e : 0) + Kb*(u_sat - u_raw)
//
// `intg` accumulates the raw error; Ki (continuous gain) is scaled by Ts
// when the output is computed.
class MotorPI
{
  public:
    struct Config
    {
        float Kp;    // proportional gain
        float Ki;    // continuous integral gain [1/s]
        float Ts;    // sample period [s]
        float V_max; // symmetric output limit [V]
        float Kb;    // anti-windup back-calculation gain
    };

    explicit MotorPI(const Config& cfg) : m_cfg(cfg)
    {
    }

    void reset()
    {
        m_intg = 0.0f;
        m_e = 0.0f;
        m_u = 0.0f;
    }

    // One control step; returns the saturated voltage command.
    float update(float ref, float meas)
    {
        m_e = ref - meas;

        const float u_raw = m_cfg.Kp * m_e + m_cfg.Ki * m_cfg.Ts * m_intg;
        const float u_sat = clampf(u_raw, -m_cfg.V_max, m_cfg.V_max);
        const float clip = u_sat - u_raw;

        // Stop accumulating error while saturated; bleed the integrator via Kb.
        if (fabsf(u_raw) < m_cfg.V_max)
            m_intg += m_e + m_cfg.Kb * clip;
        else
            m_intg += m_cfg.Kb * clip;

        m_u = u_sat;
        return m_u;
    }

    float error() const
    {
        return m_e;
    }
    float integrator() const
    {
        return m_intg;
    }
    float command() const
    {
        return m_u;
    }

  private:
    static float clampf(float x, float lo, float hi)
    {
        return (x < lo) ? lo : (x > hi) ? hi : x;
    }

    Config m_cfg;
    float m_intg = 0.0f;
    float m_e = 0.0f;
    float m_u = 0.0f;
};
