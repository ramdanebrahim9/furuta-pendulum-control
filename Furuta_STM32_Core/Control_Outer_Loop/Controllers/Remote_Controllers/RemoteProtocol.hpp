// Constants shared by both ends of the STM32 <-> Pi serial link.
//
// Every control tick the STM32 sends one fixed-size request and the Pi
// answers with one fixed-size reply:
//
//   request  (STM32 -> Pi), 21 bytes:
//       [ mode : 1 byte | state x : 4 floats | u_in : 1 float ]
//   reply    (Pi -> STM32), 5 bytes:
//       [ RX_MAGIC : 1 byte | torque tau : 1 float ]
//
// The Pi side (main.py) must use the same values.
#pragma once
#include <cstddef>
#include <cstdint>

namespace furuta {
namespace remote {

// First byte of every request. Tells the Pi which solver to run, or that a
// special command follows.
enum class SolverMode : uint8_t
{
    // Normal operation: run this solver.
    cMPC           = 0x01, // constrained MPC
    NMPC_Stabilize = 0x02, // nonlinear MPC, hold-up
    NMPC_Swingup   = 0x03, // nonlinear MPC, swing-up

    // Reset commands. Sent for exactly one tick in place of the normal mode
    // when the controller is re-entered mid-run (for example after a fall and
    // recovery). The Pi clears that solver's internal state, including its
    // warm start, before using the next state.
    cMPC_Reset           = 0x11,
    NMPC_Stabilize_Reset = 0x12,
    NMPC_Swingup_Reset   = 0x13,

    // Reset every solver on the Pi. Sent once when the STM32 leaves
    // Pi-controlled operation entirely (the STOP state).
    STOP_All = 0x20
};

// First byte of every reply. A frame that does not start with it is
// misaligned or stitched together from two replies, and is discarded.
static constexpr uint8_t RX_MAGIC = 0xA5;

static constexpr size_t RX_FRAME_SIZE = 1 + sizeof(float);     // magic + tau
static constexpr size_t TX_FRAME_SIZE = 1 + 5 * sizeof(float); // mode + 4 states + u_in

} // namespace remote
} // namespace furuta
