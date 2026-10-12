// Sample times of the two control loops, as floats.
//
// The rates are defined once, in log_config.hpp. Anything that needs a loop's
// sample time (controllers, filters, estimators) takes it from here instead of
// writing its own number, so changing a rate there updates everything.
#pragma once
#include "log_config.hpp"

// Inner current loop: every 100 us (10 kHz).
constexpr float SAMPLE_TIME_INNER_S = CONTROL_INTERVAL_US * 1e-6f;

// Outer pendulum loop: every 5 ms (200 Hz).
constexpr float SAMPLE_TIME_OUTER_S = PENDL_INTERVAL_US * 1e-6f;
