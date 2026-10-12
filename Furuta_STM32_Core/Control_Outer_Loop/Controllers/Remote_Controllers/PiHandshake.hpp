// Startup check that the Pi is alive and answering.
//
// Run once, right before an experiment starts, and separate from the
// per-tick protocol. If a Pi-based controller is selected but the Pi script
// is not running (or the cable is unplugged), the control loop would only
// find out mid-run as zero torque. This catches it before the experiment
// begins.
//
//   STM32 -> Pi : PING_BYTE
//   Pi -> STM32 : PONG_BYTE
//
// The call blocks while waiting, so it must run from normal code, never from
// an interrupt. It uses the blocking UART calls and leaves the interrupt-
// driven receive used by Controller_Remote untouched.
#pragma once
#include <cstdint>
#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart4; // UART4, the link to the Pi

namespace furuta {

class PiHandshake
{
  public:
    static constexpr uint8_t PING_BYTE = 0x50; // 'P'
    static constexpr uint8_t PONG_BYTE = 0x4B; // 'K'

    // Returns true only if the Pi answers with PONG_BYTE within timeout_ms.
    // A timeout, a wrong byte and a UART error all return false.
    static bool waitForPi(uint32_t timeout_ms = 500)
    {
        uint8_t tx = PING_BYTE;
        if (HAL_UART_Transmit(&huart4, &tx, 1, timeout_ms) != HAL_OK)
            return false;

        uint8_t rx = 0;
        if (HAL_UART_Receive(&huart4, &rx, 1, timeout_ms) != HAL_OK)
            return false;

        return rx == PONG_BYTE;
    }
};

} // namespace furuta
