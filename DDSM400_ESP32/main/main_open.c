#include "motor_mode.h"
#include "DDSM400.h"
#include <math.h>

static bool drive(uint8_t id, float value, uint8_t accel_time)
{
    (void)accel_time; /* Acceleration applies only to speed loop. */
    return isfinite(value) && value >= -30000.0f && value <= 30000.0f &&
           set_open_loop(id, (int16_t)value);
}

static bool stop_mode(uint8_t id) { return set_open_loop(id, 0); }

const motor_mode_t motor_mode_open = {
    .name = "open_loop", .label = "Open loop raw", .note = "Raw output; speed is not regulated.",
    .max_value = 30000.0f, .initial_value = 2000.0f, .step = 500.0f,
    .position_target = false, .enable = servo_on_open_loop,
    .drive = drive, .stop = stop_mode
};
