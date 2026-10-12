// Manual state requests from the debug CLI ("stop" and "go").
//
// The CLI writes a request into the mailbox below; the outer loop reads and
// clears it once per tick and decides whether to honour it. The rules about
// which request is legal from which state live in StateMachine.cpp, so the
// whole transition table is in one place (see State/SystemState.hpp).
//
// Atomics are used because the CLI and the outer-loop timer interrupt run in
// different contexts.
#pragma once
#include <atomic>
#include <cstdint>
#include "../State/SystemState.hpp"

namespace furuta
{

// There is no request for HOLD_UP: it is only reached from SWING_UP, once the
// pendulum is actually upright.
enum class SystemStateRequest
{
    NONE,
    STOP,
    SWING_UP, // "go"
};

inline std::atomic<SystemStateRequest> g_system_state_request{SystemStateRequest::NONE};

// What happened to a request. The outer loop runs in an interrupt and must not
// print, so it publishes the outcome here and the CLI prints it from normal
// code.
enum class SystemStateOutcome
{
    NONE,
    APPLIED,  // the state changed
    ALREADY,  // already in the requested state
    REJECTED, // not allowed from the current state
};

struct SystemStateResult
{
    SystemStateOutcome outcome = SystemStateOutcome::NONE;
    SystemState requested = SystemState::STOP;
};

// The outer loop writes the result and then increments the sequence number;
// the CLI re-reads the result whenever the number changes.
inline SystemStateResult g_system_state_result{};
inline std::atomic<uint32_t> g_system_state_result_seq{0};

} // namespace furuta
