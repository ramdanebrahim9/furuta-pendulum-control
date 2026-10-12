// Motor drive output: turns the inner loop's voltage command into a PWM duty
// cycle and a direction signal for the motor driver.
//
// The magnitude of the voltage sets the duty cycle (as a fraction of the
// supply voltage v_max), and its sign sets the direction pin. The
// implementation is hardware specific (timer and GPIO setup) and is not part
// of this showcase.
#pragma once

class PWMEngine
{
  public:
    // Starts the PWM output at zero duty.
    static bool start();

    // Zero duty and stops the output (emergency stop).
    static void kill();

    // Applies a voltage command [V]. |v| / v_max becomes the duty cycle; the
    // sign selects the direction.
    static void set_duty(float v, float v_max = 12.0f);
};
