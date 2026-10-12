#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    // Clears PI, EKF and filter state before a new run.
    void control_reset(void);

    // One inner-loop iteration, every 100 us (10 kHz) (called from the control timer interrupt).
    void control_loop_step(void);

#ifdef __cplusplus
}
#endif
