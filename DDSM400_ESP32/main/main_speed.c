#include "motor_mode.h"
#include "DDSM400.h"
#include <math.h>

static bool drive(uint8_t id, float value, uint8_t accel_time)
{
    if (!isfinite(value) || fabsf(value) > 380.0f) return false;
    return set_speed_soft(id, fabsf(value), accel_time, value >= 0.0f);
}

static bool stop_mode(uint8_t id) { return set_speed(id, 0.0f, true); }
static bool enable_mode(uint8_t id) { return servo_on_mode(id, DDSM_SPEED); }

const motor_mode_t motor_mode_speed = {
    .name = "speed", .label = "Speed (RPM)", .note = "Speed loop; target is RPM.",
    .max_value = 380.0f, .initial_value = 10.0f, .step = 1.0f,
    .position_target = false, .enable = enable_mode,
    .drive = drive, .stop = stop_mode
};
