#ifndef DDSM400_H
#define DDSM400_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Odometry calibration. One protocol lap contains 32768 position counts.
 * Measure the number of reported laps for one physical wheel revolution and
 * override DDSM_ODOM_LAPS_PER_WHEEL_TURN when the hardware result differs.
 */
#ifndef DDSM_ODOM_LAPS_PER_WHEEL_TURN
#define DDSM_ODOM_LAPS_PER_WHEEL_TURN 1.92
#endif
#define DDSM_POSITION_COUNTS_PER_TURN 32768LL
#define DDSM_ODOM_COUNTS_PER_WHEEL_TURN \
    (DDSM_ODOM_LAPS_PER_WHEEL_TURN * (double)DDSM_POSITION_COUNTS_PER_TURN)
/* Position-loop commands still use one 0..32767 angular revolution. */
#define DDSM_RAW_TURNS_PER_WHEEL_TURN 1.0f

/* DDSM400 command protocol. Frames are always 10 bytes, big-endian. */
enum { DDSM_SIZE=10, DDSM_BAUD=38400, DDSM_ID=0, DDSM_OPCODE=1,
       DDSM_VALUE_H=2, DDSM_VALUE_L=3, DDSM_CURRENT_H=4, DDSM_CURRENT_L=5,
       DDSM_ACCEL=6, DDSM_BRAKE_OR_TEMP=7, DDSM_FAULT=8, DDSM_CRC=9 };
enum { DDSM_DRIVE=0x64, DDSM_STATUS=0x65, DDSM_ODOM_QUERY=0x74,
       DDSM_ODOM_REPLY=0x75, DDSM_MODE_QUERY=0x75, DDSM_MODE_REPLY=0x76,
       DDSM_SWITCH=0xA0, DDSM_SWITCH_REPLY=0xA1 };
enum { DDSM_OPEN=0, DDSM_CURRENT=1, DDSM_SPEED=2, DDSM_POSITION=3,
       DDSM_ENABLE=8, DDSM_DISABLE=9 };
enum { DDSM_HALL=1, DDSM_OVERCURRENT=2, DDSM_STALL=8, DDSM_OVERHEAT=16,
       DDSM_DISCONNECTED=32, DDSM_VOLTAGE=64 };
enum { DDSM_ALARM_NONE=0, DDSM_ALARM_OVERCURRENT=1,
       DDSM_ALARM_OVERLOAD=2, DDSM_ALARM_OVERTEMPERATURE=3,
       DDSM_ALARM_MOTOR_CONNECT=4 };

typedef uint8_t ddsm_frame[DDSM_SIZE];

typedef struct {
    uint8_t id, opcode, mode, accel, temperature, faults;
    int16_t speed_x10, current_raw;
    int32_t turns;
    uint16_t position_raw;
} ddsm_feedback;

/* Keep one state object per motor when using odometry. */
typedef struct {
    int64_t origin_counts;
    int64_t last_counts;
    uint64_t travelled_counts;
    int32_t last_turns;
    uint16_t last_position;
    bool initialized;
} ddsm_distance_state;

uint8_t ddsm_crc8(const uint8_t *data, unsigned length);
bool ddsm_mode(ddsm_frame f, uint8_t id, uint8_t mode);
bool ddsm_speed(ddsm_frame f, uint8_t id, int rpm_x10, uint8_t accel);
bool ddsm_drive_raw(ddsm_frame f, uint8_t id, int16_t value, uint8_t accel, bool brake);
bool ddsm_query(ddsm_frame f, uint8_t id, uint8_t opcode);
/* Use only with one powered motor; send the same frame five times as Waveshare specifies. */
bool ddsm_set_id_frame(ddsm_frame f, uint8_t new_id);
bool ddsm_decode(const ddsm_frame f, uint8_t expected_id, uint8_t expected_opcode,
                 ddsm_feedback *out);

/*
 * One task must own the UART and call these control APIs serially.
 * exchange() must transmit one complete frame, wait for a valid response,
 * validate ID/opcode/CRC via ddsm_decode(), and return false on timeout/error.
 */
typedef bool (*servo_exchange_fn)(const ddsm_frame, uint8_t, ddsm_feedback *);
void servo_init(servo_exchange_fn exchange);
/* Enable the motor and select one of DDSM_OPEN..DDSM_POSITION. */
bool servo_on_mode(uint8_t id, uint8_t mode);
/* Default enable mode is Open Loop. */
bool servo_on(uint8_t id);
bool servo_on_open_loop(uint8_t id);
bool servo_on_current(uint8_t id);
bool servo_on_position(uint8_t id);
bool servo_off(uint8_t id);
bool servo_is_enabled(uint8_t id);

/* Magnitude in RPM (0..380), rounded to 0.1 RPM. forward=true is positive RPM. */
bool set_speed(uint8_t id, float speed, bool forward);
/* accel_time is written to DATA[6], in ms per RPM; 0 selects motor default. */
bool set_speed_soft(uint8_t id, float speed, uint8_t accel_time, bool forward);
/* Signed raw open-loop drive command; no RPM conversion or acceleration ramp. */
bool set_open_loop(uint8_t id, int16_t raw_value);
/* Send a signed 0x64 value in the already selected mode. */
bool set_mode_raw(uint8_t id, int16_t raw_value);
/* stop() sets speed to zero and leaves the motor enabled. */
bool stop(uint8_t id);
/* Sends DISABLE. This is not a safety-rated mechanical emergency stop. */
bool stop_emergency(uint8_t id);

/* Getters return the latest valid 0x65 actual frame cached by a drive command. */
bool get_feedback(uint8_t id, ddsm_feedback *feedback);
bool get_speed(uint8_t id, float *speed_rpm);
bool get_current(uint8_t id, float *current_a);
bool get_temperature(uint8_t id, uint8_t *temperature_c);
bool get_fault_flags(uint8_t id, uint8_t *fault_flags);
/* Query the 0x74 odometry frame for rotor position within one turn (0..32767). */
bool get_position_raw(uint8_t id, uint16_t *position_raw);
/* 0: none, 1: overcurrent, 2: overload/stall, 3: overtemperature, 4: other hardware fault. */
uint8_t GetAlarmType(uint8_t id);

void distance_reset(ddsm_distance_state *state);
int64_t ddsm_odometry_counts(const ddsm_feedback *feedback);
int64_t distance_displacement_counts(const ddsm_distance_state *state);
double ddsm_counts_to_m(int64_t counts, float wheel_diameter_m);
bool get_distance(uint8_t id, float wheel_diameter_m,
                  ddsm_distance_state *state,
                  float *travelled_m, float *displacement_m,
                  ddsm_feedback *feedback);

#endif
