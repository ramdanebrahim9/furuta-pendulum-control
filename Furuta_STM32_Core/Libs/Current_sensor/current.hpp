// Motor current measurement.
//
// A current sensor on the motor supply feeds an ADC, which the DMA keeps
// updated in the background, so reading the current is just reading a
// register. The inner loop reads it every 100 us.
//
// The sensor output is a voltage with an offset at zero current. calibrate()
// averages the idle reading to find that offset, and readCurrent() subtracts
// it and applies the sensor gain. The implementation is hardware specific and
// is not part of this showcase.
#pragma once

class CurrentSensor
{
  public:
    // Starts the ADC and the DMA that keeps the reading up to date.
    bool start();

    // Measures the zero-current offset. Call with the motor idle.
    void calibrate();

    // Motor current [A], positive in the positive torque direction.
    float readCurrent() const;
};
