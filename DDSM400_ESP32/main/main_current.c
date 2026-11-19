#include "motor_mode.h"
#include "DDSM400.h"
#include <math.h>

/* DDSM400 command scale: signed -32767..32767 represents -4..4 A. */
static bool drive(uint8_t id, float amps, uint8_t accel_time)
{
    (void)accel_time;
    if (!isfinite(amps) || fabsf(amps) > 1.0f) return false;
    int16_t raw = (int16_t)lroundf(amps * (32767.0f / 4.0f));
    return set_mode_raw(id, raw);
}

static bool stop_mode(uint8_t id) { return set_mode_raw(id, 0); }

const motor_mode_t motor_mode_current = {
    .name = "current", .label = "Current (A)", .note = "Current loop; target is torque-producing current, up to 1 A.",
    .max_value = 1.0f, .initial_value = 0.10f, .step = 0.01f,
    .position_target = false, .enable = servo_on_current,
    .drive = drive, .stop = stop_mode
};
