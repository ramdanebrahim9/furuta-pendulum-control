// Records why the system entered SystemState::DEAD, so the end-of-run log can
// state the cause instead of just "the system died".
//
// SYSTEM_ALIVE is the default. Like DEAD itself, a recorded reason survives
// the start of a new experiment and is only cleared by an MCU reset.
#pragma once

namespace furuta
{

enum class DeathReason
{
    SYSTEM_ALIVE, // has not died
    ARM_LIMIT,    // the arm passed its mechanical limit (cable twist)
};

} // namespace furuta
