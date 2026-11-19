#include "motor_mode.h"
#include "DDSM400.h"
#include <math.h>

#define POSITION_COUNTS_PER_TURN 32768

/* DDSM position command: 0..32767 maps directly to 0..360 motor degrees. */
static bool drive(uint8_t id, float degrees, uint8_t accel_time)
{
    (void)accel_time;
    if (!isfinite(degrees) ||
        fabsf(degrees) > 180.0f) return false;
    float wrapped = fmodf(degrees + 360.0f, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    int16_t raw = (int16_t)lroundf(wrapped * (32767.0f / 360.0f));
    return set_mode_raw(id, raw);
}

/* The main task cancels future waypoints. Sending zero would return to origin. */
static bool stop_mode(uint8_t id) { (void)id; return true; }

const motor_mode_t motor_mode_position = {
    .name = "position", .label = "Wheel position (degrees)",
    .note = "Direct motor position command: 0–180 degrees. One command is sent once; Hold does not resend it.",
    .max_value = 180.0f, .initial_value = 15.0f, .step = 1.0f,
    .position_target = true, .enable = servo_on_position,
    .drive = drive, .stop = stop_mode
};
