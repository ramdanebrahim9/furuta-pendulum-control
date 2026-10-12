// Log stream for the outer pendulum loop (one sample every 5 ms).
//
// Works like CtrlLog: the loop pushes one sample per tick, a double buffer
// in shared memory decouples it from the SD card writer on the second core.
// The two streams are separate on purpose, with their own buffer and their
// own file. What each slot means is listed in entity_schema.hpp.
#pragma once
#include <array>
#include "log_config.hpp"

class PendlLog
{
  public:
    static constexpr uint16_t N_ENTITIES = 14;

    struct Sample
    {
        std::array<EntityValue, N_ENTITIES> entity{};
    };

    // Adds one sample to the stream. Called once per outer-loop tick.
    static void FillBuffer(const Sample& sample);
};
