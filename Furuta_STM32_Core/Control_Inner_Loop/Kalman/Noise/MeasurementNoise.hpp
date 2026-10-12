// Measurement noise variance R for the scalar current measurement.
//
//   fast_track : the reference just moved, so R is small and the filter
//                snaps onto the sensor to avoid lag.
//   otherwise  : R follows the local measurement standard deviation, so a
//                noisy stretch is smoothed harder and a quiet one tracked
//                tighter.
#pragma once

float EKF_measurement_noise(bool fast_track, float meas_std);
