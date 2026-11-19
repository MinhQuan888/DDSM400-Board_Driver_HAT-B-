#ifndef MOTOR_MODE_H
#define MOTOR_MODE_H

#include <stdbool.h>
#include <stdint.h>

/* Each module supplies one mode; main.c owns UART and calls it serially. */
typedef struct {
    const char *name;
    const char *label;
    const char *note;
    float max_value;
    float initial_value;
    float step;
    bool position_target;
    bool (*enable)(uint8_t id);
    bool (*drive)(uint8_t id, float signed_value, uint8_t accel_time);
    bool (*stop)(uint8_t id);
} motor_mode_t;

extern const motor_mode_t motor_mode_open;
extern const motor_mode_t motor_mode_speed;
extern const motor_mode_t motor_mode_current;
extern const motor_mode_t motor_mode_position;

#ifndef DDSM_APP_MODE
#define DDSM_APP_MODE 0
#endif
#if DDSM_APP_MODE == 0
#define ACTIVE_MOTOR_MODE motor_mode_open
#elif DDSM_APP_MODE == 1
#define ACTIVE_MOTOR_MODE motor_mode_current
#elif DDSM_APP_MODE == 2
#define ACTIVE_MOTOR_MODE motor_mode_speed
#elif DDSM_APP_MODE == 3
#define ACTIVE_MOTOR_MODE motor_mode_position
#else
#error "DDSM_APP_MODE must be 0=open, 1=current, 2=speed or 3=position"
#endif

#endif
