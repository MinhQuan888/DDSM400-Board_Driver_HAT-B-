#ifndef WHEEL_SYNC_H
#define WHEEL_SYNC_H

#include <stdbool.h>

typedef struct {
    float id2_scale;
    float id1_scale;
    float id2_rpm;
    float id1_rpm;
    float integral;
    unsigned samples;
    unsigned stalled_samples;
    unsigned no_motion_samples;
} wheel_sync_state;

void wheel_sync_reset(wheel_sync_state *state);
/* Returns false when one wheel keeps moving while the other is stationary. */
bool wheel_sync_update(wheel_sync_state *state, float id2_rpm, float id1_rpm);

#endif
