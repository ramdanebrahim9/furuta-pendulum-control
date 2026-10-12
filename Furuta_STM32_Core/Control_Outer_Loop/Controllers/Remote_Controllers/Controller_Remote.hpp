// Controller that runs on the Raspberry Pi and is reached over UART.
//
// The hardest controllers (constrained MPC and nonlinear MPC) need a
// numerical solver that is too heavy for the STM32. They run on a Pi 5
// instead. From the outer loop this looks like any other controller, but
// compute() talks to the Pi over a serial link rather than evaluating a
// formula. One class serves all three Pi solvers; the SolverMode tells the
// Pi which one to run.
//
// How one tick works (every 5 ms, called from the outer-loop timer):
//   1. Collect the reply to the previous tick's request, if it has arrived.
//   2. Send the current state to the Pi (non-blocking).
//   3. Return the torque the Pi computed for the previous state.
//
// One-tick delay (intentional): the torque applied now was solved from the
// state one tick ago. The STM32 never waits for the Pi, so the 5 ms timing
// stays exact whatever the Pi is doing. The solvers on the Pi account for
// the delay by predicting one step ahead.
//
// The reply is received by interrupt (onUartRxComplete()), which stores the
// torque and raises a flag for the next compute() to read.
//
// Wire format: RemoteProtocol.hpp.
// Overview of the whole Pi link: README.md in this folder.
#pragma once
#include <cstdint>
#include <Eigen/Dense>
#include "RemoteProtocol.hpp"

namespace furuta {

class Controller_Remote
{
  public:
    static constexpr int NX = 4;
    using StateVec = Eigen::Matrix<float, NX, 1>;

    using SolverMode = remote::SolverMode;
    static constexpr uint8_t RX_MAGIC = remote::RX_MAGIC;
    static constexpr size_t RX_FRAME_SIZE = remote::RX_FRAME_SIZE;

    // reset_hold_ticks > 0 adds a quiet period after each reset command, used
    // by the swing-up solver whose first solve after a reset is slow (60-85 ms).
    // During the hold the torque is 0 and nothing is sent. The Pi's single
    // answer to the reset is kept but not applied until the hold ends, because
    // it was planned for the state at that moment, not earlier.
    // 0 (default): no hold.
    explicit Controller_Remote(SolverMode mode, uint16_t reset_hold_ticks = 0);

    // Sends the state to the Pi and returns the torque it computed for the
    // previous state (see the one-tick delay above). Does not block.
    //   x_est  : estimated state
    //   u_meas : torque measured on the motor this tick [N.m]
    float compute(const StateVec& x_est, float u_meas);

    // Value sent in the request's u_in slot; it depends on the solver:
    //   cMPC, NMPC_Stabilize : u_meas, the measured torque. Their problem is
    //       in increment form and needs the previous torque as part of its state.
    //   NMPC_Swingup : the torque already committed for the coming 5 ms. Its
    //       solver predicts one step ahead to cover the delay, and this class
    //       knows that torque exactly, including holds and missed replies.
    float uplinkInput(float u_meas) const;

    // Clears the link state. Called at the start of every experiment.
    void reset();

    // Makes the next compute() send this solver's reset command instead of
    // the normal request, so the Pi clears its internal state (including the
    // warm start). Call on every re-entry into this controller during a run,
    // such as after a fall and recovery, not only at the start.
    void requestPiReset() { m_send_reset = true; }

    // False if the Pi's reply was not there in time on the last compute().
    // The outer loop uses this to decide whether to stop the experiment.
    bool lastRxOk() const { return m_last_rx_ok; }

    // Called from the UART receive interrupt. Accepts the reply only if it
    // starts with RX_MAGIC; a bad frame is dropped and counted as a missed
    // reply. Returns true if the frame was accepted.
    bool onUartRxComplete();

    // Tells the Pi to reset all its solvers. Sent once when the STM32 leaves
    // Pi-controlled operation (the STOP state), independent of which
    // instance was running. Does not wait for the answer.
    static void sendPiStopAll();

    // Receive buffer, shared by all instances: only one is active at a time.
    static uint8_t s_rx_buf[RX_FRAME_SIZE];

  private:
    SolverMode m_mode;
    bool m_send_reset = false;     // set by requestPiReset(), consumed by the next compute()
    uint16_t m_reset_hold_ticks;   // length of the hold after a reset, 0 = none
    uint16_t m_hold_left = 0;      // quiet ticks still to go in the current hold

    // One-tick-delay handshake
    volatile bool m_tau_ready = false; // set by the receive interrupt, cleared by compute()
    volatile float m_next_u = 0.0f;    // newest torque from the Pi, not yet applied
    float m_prev_u = 0.0f;             // torque currently applied to the motor
    bool m_last_rx_ok = true;          // false = missed reply on the last compute()

    // The first compute() after a reset cannot have a reply yet, because it
    // sends the first request. This flag stops that first call being counted
    // as a missed reply.
    bool m_has_run_once = false;
};

} // namespace furuta
