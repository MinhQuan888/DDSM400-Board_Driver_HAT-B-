#include "DDSM400.h"
#include "motor_mode.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t mock_faults;
static uint8_t last_selected_mode;
static int32_t mock_odom_turns;
static uint16_t mock_odom_position;
static ddsm_frame last_tx;
static bool mock_exchange(const ddsm_frame tx, uint8_t expected, ddsm_feedback *out)
{
    memcpy(last_tx, tx, sizeof(last_tx));
    uint8_t frame[10] = {0};
    frame[0] = tx[0];
    frame[1] = expected;
    if (expected == DDSM_SWITCH_REPLY) {
        frame[2] = tx[2];
        if (tx[2] <= DDSM_POSITION) {
            last_selected_mode = tx[2];
        }
    } else if (expected == DDSM_STATUS) {
        frame[2] = 0x00; frame[3] = 0x63; /* 9.9 RPM */
        frame[4] = 0x02; frame[5] = 0xB0; /* raw current 688 */
        frame[6] = 1; frame[7] = 29; frame[8] = mock_faults;
    } else if (expected == DDSM_ODOM_REPLY) {
        uint32_t turns = (uint32_t)mock_odom_turns;
        frame[2] = (uint8_t)(turns >> 24);
        frame[3] = (uint8_t)(turns >> 16);
        frame[4] = (uint8_t)(turns >> 8);
        frame[5] = (uint8_t)turns;
        frame[6] = (uint8_t)(mock_odom_position >> 8);
        frame[7] = (uint8_t)mock_odom_position;
        frame[8] = mock_faults;
    }
    frame[9] = ddsm_crc8(frame, 9);
    return ddsm_decode(frame, tx[0], expected, out);
}

int main(void)
{
    const uint8_t sample[9] = {0x02,0x65,0x00,0x63,0x02,0xB0,0x01,0x1D,0x00};
    assert(ddsm_crc8(sample, 9) == 0xA0);

    ddsm_frame frame;
    assert(ddsm_speed(frame, 2, -125, 10));
    assert(frame[0] == 2 && frame[1] == 0x64 && frame[2] == 0xFF && frame[3] == 0x83);
    assert(frame[9] == ddsm_crc8(frame, 9));
    assert(!ddsm_speed(frame, 2, 3801, 0));

    servo_init(mock_exchange);
    assert(servo_on(2));
    assert(servo_is_enabled(2));
    assert(last_selected_mode == DDSM_OPEN);
    assert(!servo_on_mode(2, DDSM_ENABLE));
    assert(set_open_loop(2, 2000));

    assert(motor_mode_speed.enable(2));
    assert(last_selected_mode == DDSM_SPEED);
    assert(set_speed(2, 12.5f, true));

    float rpm = 0, current = 0;
    uint8_t temperature = 0, faults = 0;
    ddsm_feedback feedback;
    assert(get_feedback(2, &feedback));
    assert(get_speed(2, &rpm) && fabsf(rpm - 9.9f) < 0.001f);
    assert(get_current(2, &current) && fabsf(current - (688.0f * 4.0f / 32767.0f)) < 0.0001f);
    assert(get_temperature(2, &temperature) && temperature == 29);

    mock_faults = DDSM_OVERCURRENT;
    assert(!set_speed(2, 10.0f, true));
    assert(!servo_is_enabled(2));
    assert(get_fault_flags(2, &faults) && faults == DDSM_OVERCURRENT);
    assert(GetAlarmType(2) == DDSM_ALARM_OVERCURRENT);

    mock_faults = 0;
    assert(servo_on_open_loop(2));
    assert(last_selected_mode == DDSM_OPEN);
    assert(set_open_loop(2, -2000));
    assert(last_tx[1] == DDSM_DRIVE && last_tx[2] == 0xF8 &&
           last_tx[3] == 0x30 && last_tx[6] == 0 && last_tx[7] == 0);
    assert(last_tx[9] == ddsm_crc8(last_tx, 9));

    assert(motor_mode_speed.drive(2, -12.5f, 7));
    assert(last_tx[2] == 0xFF && last_tx[3] == 0x83 && last_tx[6] == 7);

    assert(motor_mode_current.enable(2));
    assert(last_selected_mode == DDSM_CURRENT);
    assert(motor_mode_current.drive(2, 1.0f, 0));
    assert(last_tx[2] == 0x20 && last_tx[3] == 0x00);
    assert(!motor_mode_current.drive(2, 1.01f, 0));

    assert(motor_mode_position.enable(2));
    assert(last_selected_mode == DDSM_POSITION);
    assert(motor_mode_position.drive(2, 90.0f, 0));
    assert(last_tx[2] == 0x20 && last_tx[3] == 0x00);
    assert(motor_mode_position.drive(2, -90.0f, 0));
    assert(last_tx[2] == 0x5F && last_tx[3] == 0xFF);
    assert(!motor_mode_position.drive(2, 11521.0f, 0));
    uint16_t position_raw = 0xffffU;
    assert(get_position_raw(2, &position_raw));
    assert(position_raw == 0U);

    ddsm_distance_state distance = {0};
    float travelled_m = 0.0f, displacement_m = 0.0f;
    const float wheel_diameter_m = 0.0745f;
    const float circumference_m = 3.14159265358979323846f * wheel_diameter_m;

    mock_odom_turns = 0;
    mock_odom_position = 32760;
    assert(get_distance(2, wheel_diameter_m, &distance,
                        &travelled_m, &displacement_m, NULL));
    mock_odom_turns = 1;
    mock_odom_position = 8;
    assert(get_distance(2, wheel_diameter_m, &distance,
                        &travelled_m, &displacement_m, NULL));
    assert(fabsf(displacement_m -
                 circumference_m * 16.0f /
                 (float)DDSM_ODOM_COUNTS_PER_WHEEL_TURN) < 1e-6f);

    distance_reset(&distance);
    mock_odom_turns = 0;
    mock_odom_position = 0;
    assert(get_distance(2, wheel_diameter_m, &distance,
                        &travelled_m, &displacement_m, NULL));
    int64_t one_wheel_turn_counts =
        (int64_t)llround(DDSM_ODOM_COUNTS_PER_WHEEL_TURN);
    mock_odom_turns = (int32_t)(one_wheel_turn_counts / 32768LL);
    mock_odom_position =
        (uint16_t)(one_wheel_turn_counts - (int64_t)mock_odom_turns * 32768LL);
    assert(get_distance(2, wheel_diameter_m, &distance,
                        &travelled_m, &displacement_m, NULL));
    assert(fabsf(displacement_m - circumference_m) < 5e-6f);
    assert(distance_displacement_counts(&distance) ==
           one_wheel_turn_counts);
    assert(distance.last_turns == mock_odom_turns &&
           distance.last_position == mock_odom_position);
    assert(fabs(ddsm_counts_to_m(one_wheel_turn_counts, wheel_diameter_m) -
                circumference_m) < 5e-6);

    distance_reset(&distance);
    mock_odom_turns = 0;
    mock_odom_position = 8;
    assert(get_distance(2, wheel_diameter_m, &distance,
                        &travelled_m, &displacement_m, NULL));
    mock_odom_turns = -1;
    mock_odom_position = 32760;
    assert(get_distance(2, wheel_diameter_m, &distance,
                        &travelled_m, &displacement_m, NULL));
    assert(fabsf(displacement_m +
                 circumference_m * 16.0f /
                 (float)DDSM_ODOM_COUNTS_PER_WHEEL_TURN) < 1e-6f);

    puts("driver tests: PASS");
    return 0;
}
