#include "Controller_Remote.hpp"
#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart4; // UART4, the link to the Pi

namespace furuta {

uint8_t Controller_Remote::s_rx_buf[RX_FRAME_SIZE];

// The constructor does not start UART reception. Several instances exist
// (one per Pi solver) and share the same UART; starting reception from each
// constructor made all but the first silently fail. compute() starts it.
Controller_Remote::Controller_Remote(SolverMode mode, uint16_t reset_hold_ticks)
    : m_mode(mode), m_reset_hold_ticks(reset_hold_ticks)
{
}

float Controller_Remote::uplinkInput(float u_meas) const
{
    switch (m_mode)
    {
    case SolverMode::NMPC_Swingup:
        return m_prev_u; // torque already committed for the coming tick
    default:
        return u_meas; // measured torque
    }
}

// Starts reception of the next reply, unless one is already part-way in.
// Aborting a part-received frame would leave half of an old reply in the
// buffer to be mixed with the next one.
static void armReceive()
{
    if (huart4.RxState == HAL_UART_STATE_READY ||
        huart4.RxXferCount == sizeof(Controller_Remote::s_rx_buf))
    {
        HAL_UART_AbortReceive_IT(&huart4);
        HAL_UART_Receive_IT(&huart4, Controller_Remote::s_rx_buf, sizeof(Controller_Remote::s_rx_buf));
    }
}

float Controller_Remote::compute(const StateVec& x_est, float u_meas)
{
    // Quiet period after a reset (see the constructor): torque 0, nothing
    // sent, any early reply stays unread. A new reset request ends the hold.
    if (m_hold_left > 0 && !m_send_reset)
    {
        --m_hold_left;
        return 0.0f;
    }

    // 1. Collect the Pi's reply to the previous tick, if it arrived.
    if (m_tau_ready)
    {
        m_prev_u = m_next_u;
        m_tau_ready = false;
        m_last_rx_ok = true;
    }
    else if (!m_has_run_once)
    {
        // Very first call: no request has been sent yet, so there is nothing
        // to wait for. Not a missed reply.
        m_last_rx_ok = true;
    }
    else
    {
        // Missed reply. Keep the last torque and let the caller decide
        // whether to stop.
        m_last_rx_ok = false;
    }
    m_has_run_once = true;

    // 2. Send the request: mode, state, u_in. If a reset was requested, the
    // reset command replaces the mode for this one tick.
    uint8_t tx_buf[remote::TX_FRAME_SIZE];

    const bool start_hold = m_send_reset && m_reset_hold_ticks > 0;
    if (start_hold)
    {
        // Discard any stale reply so the only reply that can be applied after
        // the hold is the Pi's answer to this reset. Torque is 0 from here.
        m_tau_ready = false;
        m_prev_u = 0.0f;
    }

    if (m_send_reset)
    {
        SolverMode reset_mode = m_mode;
        switch (m_mode)
        {
        case SolverMode::cMPC:           reset_mode = SolverMode::cMPC_Reset; break;
        case SolverMode::NMPC_Stabilize: reset_mode = SolverMode::NMPC_Stabilize_Reset; break;
        case SolverMode::NMPC_Swingup:   reset_mode = SolverMode::NMPC_Swingup_Reset; break;
        default: break;
        }
        tx_buf[0] = static_cast<uint8_t>(reset_mode);
        m_send_reset = false;
    }
    else
    {
        tx_buf[0] = static_cast<uint8_t>(m_mode);
    }

    for (int i = 0; i < NX; ++i)
    {
        const float v = x_est(i);
        __builtin_memcpy(&tx_buf[1 + i * sizeof(float)], &v, sizeof(float));
    }
    const float u_in = uplinkInput(u_meas);
    __builtin_memcpy(&tx_buf[1 + NX * sizeof(float)], &u_in, sizeof(float));

    HAL_UART_Transmit(&huart4, tx_buf, sizeof(tx_buf), 10);

    // 3. Listen for this request's reply. It arrives by interrupt at some
    // point before the next tick.
    armReceive();

    // This tick was the first tick of the hold; the rest follow.
    if (start_hold)
        m_hold_left = static_cast<uint16_t>(m_reset_hold_ticks - 1);

    // 4. Apply the torque from the previous request (one-tick delay).
    return m_prev_u;
}

void Controller_Remote::reset()
{
    m_hold_left = 0;
    m_tau_ready = false;
    m_next_u = 0.0f;
    m_prev_u = 0.0f;
    m_last_rx_ok = true;
    m_has_run_once = false;
}

void Controller_Remote::sendPiStopAll()
{
    // Same frame size as a normal request so the Pi's reader needs no special
    // case. State and u_in are unused for this command and sent as zero.
    uint8_t tx_buf[remote::TX_FRAME_SIZE] = {};
    tx_buf[0] = static_cast<uint8_t>(SolverMode::STOP_All);
    HAL_UART_Transmit(&huart4, tx_buf, sizeof(tx_buf), 10);

    // The Pi answers every request, STOP_All included. Re-arm reception so
    // that answer lands in a fresh listener rather than in a stale one left
    // over from the previous tick. It is then dropped by the receive
    // callback, since no controller is active in STOP to consume it.
    armReceive();
}

bool Controller_Remote::onUartRxComplete()
{
    // A frame that does not start with the magic byte is misaligned or was
    // stitched from two replies. Drop it: the next compute() then counts it
    // as a missed reply instead of trusting a garbage torque.
    if (s_rx_buf[0] != RX_MAGIC)
        return false;

    float tau;
    __builtin_memcpy(&tau, &s_rx_buf[1], sizeof(float));

    m_next_u = tau;
    m_tau_ready = true;
    return true;
}

} // namespace furuta
