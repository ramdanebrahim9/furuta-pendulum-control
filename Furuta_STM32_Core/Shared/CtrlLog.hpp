// Log stream for the inner current loop (one sample every 100 us).
//
// The loop pushes one sample per tick with FillBuffer(). Samples collect in a
// double buffer in shared memory; the second core writes them to the SD card
// while the first fills the other half, so logging never delays the control
// loop. The buffer and the SD writer are hardware specific and are not part
// of this showcase. What each slot means is listed in entity_schema.hpp.
#pragma once
#include <array>
#include "log_config.hpp"

class CtrlLog
{
  public:
    static constexpr uint16_t N_ENTITIES = 3;

    struct Sample
    {
        std::array<EntityValue, N_ENTITIES> entity{};
    };

    // Adds one sample to the stream. Called once per inner-loop tick.
    static void FillBuffer(const Sample& sample);
};
