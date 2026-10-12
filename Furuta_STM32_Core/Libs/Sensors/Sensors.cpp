#include "Sensors.hpp"
#include "Encoder.hpp"

// The encoder instances are created by the application layer, next to their
// timer handles.
extern Encoder Encoder_Arm;
extern Encoder Encoder_Pendl;

extern "C" void sensors_sample(void)
{
    Encoder_Arm.update();
    Encoder_Pendl.update();
}

extern "C" void sensors_reset(void)
{
    Encoder_Arm.clear();
    Encoder_Pendl.clear();
}

SensorAngles sensors_read_angles(void)
{
    // Each encoder increments its seq at the end of update(). If both are
    // unchanged after the reads, no update overlapped them, so the two angles
    // belong to the same sample.
    for (;;)
    {
        const uint32_t seq_arm = Encoder_Arm.seq();
        const uint32_t seq_pendl = Encoder_Pendl.seq();

        const SensorAngles angles{
            Encoder_Arm.snapshotOutputAngleRad(),
            Encoder_Pendl.snapshotOutputAngleRad(),
        };

        if (Encoder_Arm.seq() == seq_arm && Encoder_Pendl.seq() == seq_pendl)
            return angles;
    }
}

float sensors_arm_angle_deg(void)
{
    return Encoder_Arm.snapshotOutputAngleDeg();
}

// Single values passed between the loops. The two loops run at the same
// interrupt priority, so a write cannot land in the middle of a read.
static volatile float s_current_raw = 0.0f;
static volatile float s_current_filt = 0.0f;
static volatile float s_tau_cmd = 0.0f;

void sensors_publish_current(float i_meas) { s_current_raw = i_meas; }
float sensors_read_current(void) { return s_current_raw; }

void sensors_publish_current_filt(float i_filt) { s_current_filt = i_filt; }
float sensors_read_current_filt(void) { return s_current_filt; }

void sensors_publish_tau(float tau_cmd) { s_tau_cmd = tau_cmd; }
float sensors_read_tau(void) { return s_tau_cmd; }
