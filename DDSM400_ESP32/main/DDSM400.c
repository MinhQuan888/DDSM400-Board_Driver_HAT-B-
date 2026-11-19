#include "DDSM400.h"

#include <string.h>

/*
 * Calibration from DDSM400 odometry revolutions to wheel revolutions.
 * The current test setup measured about 1/32 wheel revolution when the
 * raw odometry reported one revolution, so one wheel turn equals 32 raw turns.
 * Change to 64.0f if a controlled one-wheel-turn test confirms that ratio.
 */

static bool valid_id(uint8_t id)
{
    return id >= 1 && id <= 253;
}

uint8_t ddsm_crc8(const uint8_t *data, unsigned length)
{
    uint8_t crc = 0;

    if (data == NULL) {
        return 0;
    }

    while (length--) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; bit++) {
            crc = (uint8_t)((crc >> 1) ^ ((crc & 1U) ? 0x8cU : 0U));
        }
    }

    return crc;
}

static void frame_init(ddsm_frame frame, uint8_t id, uint8_t opcode)
{
    memset(frame, 0, DDSM_SIZE);
    frame[0] = id;
    frame[1] = opcode;
}

static bool frame_finish(ddsm_frame frame)
{
    frame[9] = ddsm_crc8(frame, 9);
    return true;
}

bool ddsm_mode(ddsm_frame frame, uint8_t id, uint8_t mode)
{
    if (frame == NULL || !valid_id(id) ||
        (mode > DDSM_POSITION && mode != DDSM_ENABLE && mode != DDSM_DISABLE)) {
        return false;
    }

    frame_init(frame, id, DDSM_SWITCH);
    frame[2] = mode;
    return frame_finish(frame);
}

bool ddsm_drive_raw(ddsm_frame frame, uint8_t id, int16_t value,
                    uint8_t accel, bool brake)
{
    if (frame == NULL || !valid_id(id)) {
        return false;
    }

    frame_init(frame, id, DDSM_DRIVE);
    frame[2] = (uint8_t)(((uint16_t)value >> 8) & 0xffU);
    frame[3] = (uint8_t)value;
    frame[6] = accel;
    frame[7] = brake ? 0xffU : 0U;
    return frame_finish(frame);
}

bool ddsm_speed(ddsm_frame frame, uint8_t id, int rpm_x10, uint8_t accel)
{
    if (rpm_x10 < -3800 || rpm_x10 > 3800) {
        return false;
    }
    return ddsm_drive_raw(frame, id, (int16_t)rpm_x10, accel, false);
}

bool ddsm_query(ddsm_frame frame, uint8_t id, uint8_t opcode)
{
    if (frame == NULL || !valid_id(id) ||
        (opcode != DDSM_MODE_QUERY && opcode != DDSM_ODOM_QUERY)) {
        return false;
    }

    frame_init(frame, id, opcode);
    return frame_finish(frame);
}

bool ddsm_set_id_frame(ddsm_frame frame, uint8_t new_id)
{
    if (frame == NULL || !valid_id(new_id)) {
        return false;
    }

    frame_init(frame, 0xaa, 0x55);
    frame[2] = 0x53;
    frame[3] = new_id;
    return frame_finish(frame);
}

static uint16_t read_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t read_s16_be(const uint8_t *data)
{
    uint16_t raw = read_u16_be(data);
    int32_t value = (raw >= 0x8000U) ? ((int32_t)raw - 65536) : raw;
    return (int16_t)value;
}

bool ddsm_decode(const ddsm_frame frame, uint8_t expected_id,
                 uint8_t expected_opcode, ddsm_feedback *out)
{
    if (frame == NULL || out == NULL || frame[0] != expected_id ||
        frame[1] != expected_opcode || frame[9] != ddsm_crc8(frame, 9)) {
        return false;
    }

    ddsm_feedback decoded = {0};
    decoded.id = expected_id;
    decoded.opcode = expected_opcode;

    switch (expected_opcode) {
    case DDSM_STATUS:
        decoded.speed_x10 = read_s16_be(frame + 2);
        decoded.current_raw = read_s16_be(frame + 4);
        decoded.accel = frame[6]; 
        decoded.temperature = frame[7];
        decoded.faults = frame[8];
        break;

    case DDSM_ODOM_REPLY:
        decoded.turns = (int32_t)read_s16_be(frame + 2) * 65536L +
                        (int32_t)read_u16_be(frame + 4);
        decoded.position_raw = read_u16_be(frame + 6);
        decoded.faults = frame[8];
        if (decoded.position_raw >= DDSM_POSITION_COUNTS_PER_TURN) {
            return false;
        }
        break;

    case DDSM_MODE_REPLY:
    case DDSM_SWITCH_REPLY:
        decoded.mode = frame[2];
        break;

    default:
        return false;
    }

    *out = decoded;
    return true;
}

static servo_exchange_fn send_frame;
static bool enabled[254];
static bool status_valid[254];
static bool fault_valid[254];
static ddsm_feedback status_cache[254];
static uint8_t fault_cache[254];

void servo_init(servo_exchange_fn exchange)
{
    send_frame = exchange;
    memset(enabled, 0, sizeof(enabled));
    memset(status_valid, 0, sizeof(status_valid));
    memset(fault_valid, 0, sizeof(fault_valid));
    memset(status_cache, 0, sizeof(status_cache));
    memset(fault_cache, 0, sizeof(fault_cache));
}

bool servo_is_enabled(uint8_t id)
{
    return valid_id(id) && enabled[id];
}

bool servo_off(uint8_t id)
{
    ddsm_frame tx;
    ddsm_feedback rx;

    if (!send_frame || !ddsm_mode(tx, id, DDSM_DISABLE)) {
        return false;
    }

    /* Lock out any further speed commands even if the acknowledgement is lost. */
    enabled[id] = false;
    status_valid[id] = false;

    return send_frame(tx, DDSM_SWITCH_REPLY, &rx);
}

bool servo_on_mode(uint8_t id, uint8_t mode)
{
    ddsm_frame tx;
    ddsm_feedback rx;

    if (!valid_id(id) || mode > DDSM_POSITION || !send_frame ||
        !ddsm_mode(tx, id, DDSM_ENABLE)) {
        return false;
    }

    enabled[id] = false;
    status_valid[id] = false;
    fault_valid[id] = false;

    if (send_frame(tx, DDSM_SWITCH_REPLY, &rx) &&
        ddsm_mode(tx, id, mode) &&
        send_frame(tx, DDSM_SWITCH_REPLY, &rx) &&
        rx.mode == mode) {
        enabled[id] = true;
        return true;
    }

    (void)servo_off(id);
    return false;
}

bool servo_on(uint8_t id)
{
    return servo_on_mode(id, DDSM_OPEN);
}

bool servo_on_open_loop(uint8_t id)
{
    return servo_on_mode(id, DDSM_OPEN);
}

bool servo_on_current(uint8_t id)
{
    return servo_on_mode(id, DDSM_CURRENT);
}

bool servo_on_position(uint8_t id)
{
    return servo_on_mode(id, DDSM_POSITION);
}

bool set_mode_raw(uint8_t id, int16_t raw_value)
{
    ddsm_frame tx;
    ddsm_feedback rx;

    if (!valid_id(id) || !send_frame || !enabled[id] ||
        !ddsm_drive_raw(tx, id, raw_value, 0, false)) {
        return false;
    }

    if (!send_frame(tx, DDSM_STATUS, &rx)) {
        status_valid[id] = false;
        fault_valid[id] = false;
        (void)servo_off(id);
        return false;
    }

    status_cache[id] = rx;
    status_valid[id] = true;
    fault_cache[id] = rx.faults;
    fault_valid[id] = true;
    if (rx.faults != 0U) {
        (void)servo_off(id);
        return false;
    }
    return true;
}

bool set_open_loop(uint8_t id, int16_t raw_value)
{
    return set_mode_raw(id, raw_value);
}

bool set_speed_soft(uint8_t id, float speed, uint8_t accel_time, bool forward)
{
    ddsm_frame tx;
    ddsm_feedback rx;

    if (!valid_id(id) || !send_frame || !enabled[id] ||
        !(speed >= 0.0f && speed <= 380.0f)) {
        return false;
    }

    int tenths = (int)(speed * 10.0f + 0.5f);
    int signed_tenths = forward ? tenths : -tenths;

    if (!ddsm_speed(tx, id, signed_tenths, accel_time)) {
        return false;
    }

    if (!send_frame(tx, DDSM_STATUS, &rx)) {
        status_valid[id] = false;
        fault_valid[id] = false;
        /* Best-effort disable: a missing reply does not prove the motor stopped. */
        (void)servo_off(id);
        return false;
    }

    status_cache[id] = rx;
    status_valid[id] = true;
    fault_cache[id] = rx.faults;
    fault_valid[id] = true;

    if (rx.faults != 0U) {
        /* Keep the fault bits for diagnostics; servo_off clears only actual cache. */
        (void)servo_off(id);
        return false;
    }

    return true;
}

bool set_speed(uint8_t id, float speed, bool forward)
{
    return set_speed_soft(id, speed, 0, forward);
}

bool stop(uint8_t id)
{
    return set_speed(id, 0.0f, true);
}

bool stop_emergency(uint8_t id)
{
    /* Protocol DISABLE only; the wheel may still coast due to mechanical inertia. */
    return servo_off(id);
}

static bool cached_status(uint8_t id, const ddsm_feedback **status)
{
    if (status == NULL || !valid_id(id) || !status_valid[id]) {
        return false;
    }

    *status = &status_cache[id];
    return true;
}

bool get_feedback(uint8_t id, ddsm_feedback *feedback)
{
    const ddsm_feedback *cached;

    if (feedback == NULL || !cached_status(id, &cached)) {
        return false;
    }

    *feedback = *cached;
    return true;
}

bool get_speed(uint8_t id, float *speed_rpm)
{
    const ddsm_feedback *status;

    if (speed_rpm == NULL || !cached_status(id, &status)) {
        return false;
    }

    *speed_rpm = (float)status->speed_x10 / 10.0f;
    return true;
}

bool get_current(uint8_t id, float *current_a)
{
    const ddsm_feedback *status;

    if (current_a == NULL || !cached_status(id, &status)) {
        return false;
    }

    /* Documented scale: signed raw -32767..32767 represents -4..4 A. */
    *current_a = (float)status->current_raw * (4.0f / 32767.0f);
    return true;
}

bool get_temperature(uint8_t id, uint8_t *temperature_c)
{
    const ddsm_feedback *status;

    if (temperature_c == NULL || !cached_status(id, &status)) {
        return false;
    }

    *temperature_c = status->temperature;
    return true;
}

bool get_fault_flags(uint8_t id, uint8_t *fault_flags)
{
    if (fault_flags == NULL || !valid_id(id) || !fault_valid[id]) {
        return false;
    }

    *fault_flags = fault_cache[id];
    return true;
}

uint8_t GetAlarmType(uint8_t id)
{
    uint8_t faults;

    if (!get_fault_flags(id, &faults) || faults == 0U) {
        return DDSM_ALARM_NONE;
    }

    if ((faults & DDSM_OVERCURRENT) != 0U) {
        return DDSM_ALARM_OVERCURRENT;
    }
    if ((faults & DDSM_STALL) != 0U) {
        return DDSM_ALARM_OVERLOAD;
    }
    if ((faults & DDSM_OVERHEAT) != 0U) {
        return DDSM_ALARM_OVERTEMPERATURE;
    }

    return DDSM_ALARM_MOTOR_CONNECT;
}

void distance_reset(ddsm_distance_state *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

bool get_position_raw(uint8_t id, uint16_t *position_raw)
{
    ddsm_frame tx;
    ddsm_feedback rx;
    if (position_raw == NULL || !send_frame ||
        !ddsm_query(tx, id, DDSM_ODOM_QUERY) ||
        !send_frame(tx, DDSM_ODOM_REPLY, &rx) || rx.faults != 0U) {
        return false;
    }
    *position_raw = rx.position_raw;
    return true;
}

int64_t ddsm_odometry_counts(const ddsm_feedback *feedback)
{
    if (feedback == NULL) {
        return 0;
    }

    /* For reverse motion, turns can be negative while position is near 32767. */
    return (int64_t)feedback->turns * DDSM_POSITION_COUNTS_PER_TURN +
           feedback->position_raw;
}

int64_t distance_displacement_counts(const ddsm_distance_state *state)
{
    if (state == NULL || !state->initialized) {
        return 0;
    }
    return state->last_counts - state->origin_counts;
}

double ddsm_counts_to_m(int64_t counts, float wheel_diameter_m)
{
    return (double)counts *
           (3.14159265358979323846 * (double)wheel_diameter_m) /
           DDSM_ODOM_COUNTS_PER_WHEEL_TURN;
}

bool get_distance(uint8_t id, float wheel_diameter_m,
                  ddsm_distance_state *state,
                  float *travelled_m, float *displacement_m,
                  ddsm_feedback *feedback)
{
    ddsm_frame tx;
    ddsm_feedback rx;

    if (!send_frame || state == NULL || travelled_m == NULL ||
        displacement_m == NULL || !(wheel_diameter_m > 0.0f) ||
        !ddsm_query(tx, id, DDSM_ODOM_QUERY) ||
        !send_frame(tx, DDSM_ODOM_REPLY, &rx) || rx.faults != 0U) {
        return false;
    }

    int64_t counts = ddsm_odometry_counts(&rx);

    if (!state->initialized) {
        state->origin_counts = counts;
        state->last_counts = counts;
        state->travelled_counts = 0;
        state->initialized = true;
    } else {
        int64_t delta = counts - state->last_counts;
        state->travelled_counts += (uint64_t)(delta < 0 ? -delta : delta);
        state->last_counts = counts;
    }

    state->last_turns = rx.turns;
    state->last_position = rx.position_raw;

    *travelled_m = (float)ddsm_counts_to_m((int64_t)state->travelled_counts,
                                           wheel_diameter_m);
    *displacement_m = (float)ddsm_counts_to_m(counts - state->origin_counts,
                                              wheel_diameter_m);

    if (feedback != NULL) {
        *feedback = rx;
    }

    return true;
}
