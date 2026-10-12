// Live shift of the arm's centre of balance.
//
// The debug CLI command "trj<deg>" (for example "trj30" or "trj-30") changes
// where the controllers think theta = 0 is, while the system is running. The
// value is absolute, not cumulative: "trj50" then "trj10" puts the centre at
// 10 degrees, not 60.
//
// The shift is applied only to the theta that is fed to the controllers. The
// estimator, the arm safety limit and the logged arm angle all keep using the
// true angle. A controller balancing "theta = 0" therefore moves the real arm
// to the shifted centre, while the death limit stays where it is.
//
// A plain atomic is enough: any value is legal in any state, and there is no
// result to report back.
#pragma once
#include <atomic>

namespace furuta
{

// Shift [rad]. Written by the CLI, read once per tick by the outer loop.
inline std::atomic<float> g_arm_reference_shift_rad{0.0f};

} // namespace furuta
