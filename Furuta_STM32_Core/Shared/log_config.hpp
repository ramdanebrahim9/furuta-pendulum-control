// Loop rates and the logging switch. The one place a rate is written down;
// SampleTimes.hpp and everything else derive from it.
#pragma once
#include <cstdint>

// Every logged value is a float. Counters, modes and flags are cast to float
// when logged (exact up to 2^24), so a log file is a plain float matrix.
using EntityValue = float;

// Base tick of the fast timer.
constexpr uint32_t TIMER_INTERVAL_US = 50;

// The inner current loop runs on every second base tick.
constexpr uint32_t CONTROL_DECIMATION = 2;
constexpr uint32_t CONTROL_INTERVAL_US = TIMER_INTERVAL_US * CONTROL_DECIMATION; // 100 us

// The outer pendulum loop.
constexpr uint32_t PENDL_INTERVAL_US = 5000; // 5 ms

// When false, neither loop pushes samples to the log, so the experiment runs
// without logging.
constexpr bool LOGGING_ENABLED = false;
