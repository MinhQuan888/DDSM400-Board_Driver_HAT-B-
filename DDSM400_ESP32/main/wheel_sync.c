#include "wheel_sync.h"

#include <math.h>
#include <stddef.h>

#define SYNC_MIN_SCALE 0.70f
#define SYNC_MAX_STEP 0.10f
#define SYNC_KP 0.45f
#define SYNC_KI 0.08f
#define SYNC_MAX_CORRECTION 0.30f
#define SYNC_DEADBAND_RPM 0.8f
#define SYNC_STALL_RPM 1.5f
#define SYNC_MOVING_RPM 5.0f
#define SYNC_STALL_SAMPLES 2U
#define SYNC_NO_MOTION_SAMPLES 4U

static float bounded(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

void wheel_sync_reset(wheel_sync_state *state)
{
    if (state == NULL) return;
    *state = (wheel_sync_state){ .id2_scale = 1.0f, .id1_scale = 1.0f };
}

bool wheel_sync_update(wheel_sync_state *state, float id2_rpm, float id1_rpm)
{
    if (state == NULL || !isfinite(id2_rpm) || !isfinite(id1_rpm)) return false;

    /* Inputs are signed wheel speeds in the requested vehicle direction. */
    if (id2_rpm < -SYNC_STALL_RPM || id1_rpm < -SYNC_STALL_RPM) return false;
    float ref = fmaxf(0.0f, id2_rpm);
    float mirror = fmaxf(0.0f, id1_rpm);
    if (state->samples == 0U) {
        state->id2_rpm = ref;
        state->id1_rpm = mirror;
    } else {
        state->id2_rpm = 0.5f * (state->id2_rpm + ref);
        state->id1_rpm = 0.5f * (state->id1_rpm + mirror);
    }
    state->samples++;

    bool stalled = (state->id2_rpm < SYNC_STALL_RPM && state->id1_rpm > SYNC_MOVING_RPM) ||
                   (state->id1_rpm < SYNC_STALL_RPM && state->id2_rpm > SYNC_MOVING_RPM);
    state->stalled_samples = stalled ? state->stalled_samples + 1U : 0U;
    if (state->stalled_samples >= SYNC_STALL_SAMPLES) return false;

    bool no_motion = state->id2_rpm < SYNC_STALL_RPM &&
                     state->id1_rpm < SYNC_STALL_RPM;
    state->no_motion_samples = no_motion ? state->no_motion_samples + 1U : 0U;
    if (state->no_motion_samples >= SYNC_NO_MOTION_SAMPLES) return false;

    if (state->samples < 2U ||
        fmaxf(state->id2_rpm, state->id1_rpm) < SYNC_MOVING_RPM) return true;

    float error_rpm = state->id2_rpm - state->id1_rpm;
    if (fabsf(error_rpm) < SYNC_DEADBAND_RPM) error_rpm = 0.0f;
    float error = error_rpm / fmaxf(fmaxf(state->id2_rpm, state->id1_rpm),
                                    SYNC_MOVING_RPM);
    state->integral = bounded(state->integral + SYNC_KI * error,
                              -SYNC_MAX_CORRECTION, SYNC_MAX_CORRECTION);
    float correction = bounded(SYNC_KP * error + state->integral,
                               -SYNC_MAX_CORRECTION, SYNC_MAX_CORRECTION);
    /* Preserve the requested limit: only slow the wheel that is moving faster. */
    float id2_target = bounded(1.0f - fmaxf(correction, 0.0f),
                               SYNC_MIN_SCALE, 1.0f);
    float id1_target = bounded(1.0f + fminf(correction, 0.0f),
                               SYNC_MIN_SCALE, 1.0f);
    state->id2_scale += bounded(id2_target - state->id2_scale,
                                -SYNC_MAX_STEP, SYNC_MAX_STEP);
    state->id1_scale += bounded(id1_target - state->id1_scale,
                                -SYNC_MAX_STEP, SYNC_MAX_STEP);
    return true;
}
