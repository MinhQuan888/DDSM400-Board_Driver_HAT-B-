#include "DDSM400.h"
#include "wheel_sync.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if !defined(CONFIG_DDSM_TX) || !defined(CONFIG_DDSM_RX)
#error "Configure DDSM motor UART TX/RX pins"
#endif

#define TAG "MOBILE"
#define MOTOR_UART UART_NUM_2
#define USB_UART UART_NUM_0
#define MOTOR_TX ((gpio_num_t)CONFIG_DDSM_TX)
#define MOTOR_RX ((gpio_num_t)CONFIG_DDSM_RX)
static int16_t open_raw = 5000;
static int16_t commanded_id1 = 0, commanded_id2 = 0;
static bool motors_ready = false;
static bool last_drive_error_enable = false;
static uint8_t active_mode = DDSM_SPEED;
static wheel_sync_state mobile_sync = {.id1_scale = 1.0f, .id2_scale = 1.0f};
static char active_motion = 'S';

static void update_pair_targets(void)
{
    int direction = active_motion == 'U' ? 1 : -1;
    commanded_id2 = (int16_t)lroundf(direction * open_raw * mobile_sync.id2_scale);
    commanded_id1 = (int16_t)lroundf(-direction * open_raw * mobile_sync.id1_scale);
}

static bool exchange(const ddsm_frame tx, uint8_t opcode, ddsm_feedback *out)
{
    uint8_t frame[DDSM_SIZE]; size_t count = 0;
    if (tx == NULL || out == NULL) return false;
    uart_flush_input(MOTOR_UART);
    if (uart_write_bytes(MOTOR_UART, (const char *)tx, DDSM_SIZE) != DDSM_SIZE) return false;
    int64_t deadline = esp_timer_get_time() + 250000;
    while (esp_timer_get_time() < deadline) {
        uint8_t byte;
        if (uart_read_bytes(MOTOR_UART, &byte, 1, pdMS_TO_TICKS(2)) != 1) continue;
        if (count == DDSM_SIZE) { memmove(frame, frame + 1, DDSM_SIZE - 1); count--; }
        frame[count++] = byte;
        if (count == DDSM_SIZE && ddsm_decode(frame, tx[0], opcode, out)) return true;
    }
    return false;
}

static void reply(const char *message)
{
    uart_write_bytes(USB_UART, message, strlen(message));
    uart_write_bytes(USB_UART, "\r\n", 2);
}

static bool command_motor(uint8_t id, const char *mode, float value)
{
    ddsm_frame frame; ddsm_feedback feedback;
    bool ok = false;
    if (strcmp(mode, "open") == 0) {
        ok = ddsm_drive_raw(frame, id, (int16_t)value, 0, false) && exchange(frame, DDSM_STATUS, &feedback);
    } else if (strcmp(mode, "current") == 0) {
        if (value < -1.0f || value > 1.0f) return false;
        ok = ddsm_drive_raw(frame, id, (int16_t)lroundf(value * 32767.0f / 4.0f), 0, false) && exchange(frame, DDSM_STATUS, &feedback);
    } else if (strcmp(mode, "speed") == 0) {
        ok = ddsm_speed(frame, id, (int)(value * 10.0f), 0) && exchange(frame, DDSM_STATUS, &feedback);
    } else if (strcmp(mode, "position") == 0) {
        if (value < 0.0f || value > 180.0f) return false;
        ok = ddsm_drive_raw(frame, id, (int16_t)lroundf(value * 32767.0f / 360.0f), 0, false) && exchange(frame, DDSM_STATUS, &feedback);
    }
    return ok;
}

static bool set_mode(uint8_t id, uint8_t mode)
{
    ddsm_frame frame; ddsm_feedback feedback;
    return ddsm_mode(frame, id, DDSM_ENABLE) && exchange(frame, DDSM_SWITCH_REPLY, &feedback) &&
           ddsm_mode(frame, id, mode) && exchange(frame, DDSM_SWITCH_REPLY, &feedback) &&
           feedback.mode == mode;
}

static bool open_command_pair(char command)
{
    ddsm_frame frame; ddsm_feedback feedback;
    int16_t id2 = 0, id1 = 0;
    last_drive_error_enable = false;
    switch (command) {
    case 'U': id2 = open_raw;  id1 = -open_raw; break; /* forward */
    case 'D': id2 = -open_raw; id1 = open_raw;  break; /* reverse */
    case 'L': id2 = -open_raw; id1 = -open_raw; break; /* turn left */
    case 'R': id2 = open_raw;  id1 = open_raw;  break; /* turn right */
    case 'S': id2 = 0;         id1 = 0;          break;
    default: return false;
    }
    if (command != active_motion) wheel_sync_reset(&mobile_sync);
    active_motion = command;
    if (command == 'U' || command == 'D') {
        update_pair_targets();
        id2 = commanded_id2;
        id1 = commanded_id1;
    }
    if (command != 'S' && !motors_ready) {
        bool ready1 = set_mode(1, DDSM_OPEN);
        bool ready2 = set_mode(2, DDSM_OPEN);
        motors_ready = ready1 && ready2;
        if (!motors_ready) {
            last_drive_error_enable = true;
            return false;
        }
    }
    commanded_id1 = id1;
    commanded_id2 = id2;
    if (!motors_ready) return true;
    bool ok2 = ddsm_drive_raw(frame, 2, id2, 0, false) && exchange(frame, DDSM_STATUS, &feedback);
    bool ok1 = ddsm_drive_raw(frame, 1, id1, 0, false) && exchange(frame, DDSM_STATUS, &feedback);
    if (!ok1 || !ok2) motors_ready = false;
    return ok1 && ok2;
}

static bool report_motor(uint8_t id, int16_t raw, ddsm_feedback *measured)
{
    ddsm_frame frame; ddsm_feedback feedback;
    char message[96];
    if (!motors_ready || !ddsm_drive_raw(frame, id, raw, 0, false) || !exchange(frame, DDSM_STATUS, &feedback)) {
        snprintf(message, sizeof(message), "FB %u ERR", (unsigned)id);
        reply(message);
        return false;
    } else {
        /* DDSM400 feedback: speed is 0.1 RPM, current is 4/32767 A per count. */
        snprintf(message, sizeof(message), "FB %u %d %d %u %u", (unsigned)id,
                 (int)feedback.speed_x10, (int)feedback.current_raw,
                 (unsigned)feedback.temperature, (unsigned)feedback.faults);
    }
    reply(message);
    if (measured != NULL) *measured = feedback;
    return true;
}

static void process_line(char *line)
{
    char command[16], arg1[16]; float value;
    if (sscanf(line, "%15s %15s %f", command, arg1, &value) < 1) return;
    for (char *p = command; *p; ++p) *p = (char)tolower((unsigned char)*p);
    if (strlen(command) == 1U) {
        char key = (char)toupper((unsigned char)command[0]);
        if (key == 'U' || key == 'D' || key == 'L' || key == 'R' || key == 'S') {
            bool ok = open_command_pair(key);
            reply(ok ? "OK drive" : (last_drive_error_enable ? "ERR enable open loop" : "ERR motor feedback"));
            return;
        }
    }
    if (strcmp(command, "help") == 0) {
        reply("OK commands: raw <0..32767>, U/D/L/R/S, enable [open|current|speed|position], drive <mode> <value>, stop, disable, status"); return;
    }
    if (strcmp(command, "raw") == 0) {
        char *end = NULL;
        long requested = strtol(line + 3, &end, 10);
        if (end == line + 3 || (*end != '\0' && !isspace((unsigned char)*end)) || requested < 0 || requested > 32767) {
            reply("ERR raw range 0..32767");
        } else {
            open_raw = (int16_t)requested;
            if (active_motion == 'U' || active_motion == 'D') update_pair_targets();
            reply("OK raw");
        }
        return;
    }
    if (strcmp(command, "enable") == 0) {
        uint8_t mode = DDSM_SPEED;
        if (strcmp(arg1, "open") == 0) mode = DDSM_OPEN;
        else if (strcmp(arg1, "current") == 0) mode = DDSM_CURRENT;
        else if (strcmp(arg1, "position") == 0) mode = DDSM_POSITION;
        if (set_mode(1, mode) && set_mode(2, mode)) { active_mode = mode; reply("OK enabled"); } else reply("ERR enable");
        return;
    }
    if (strcmp(command, "drive") == 0 && sscanf(line, "%15s %15s %f", command, arg1, &value) == 3) {
        bool ok1 = command_motor(1, arg1, -value), ok2 = command_motor(2, arg1, value);
        reply(ok1 && ok2 ? "OK drive" : "ERR drive"); return;
    }
    if (strcmp(command, "stop") == 0) {
        const char *mode_name = active_mode == DDSM_OPEN ? "open" :
                                active_mode == DDSM_CURRENT ? "current" :
                                active_mode == DDSM_POSITION ? "position" : "speed";
        bool ok1 = active_mode == DDSM_POSITION || command_motor(1, mode_name, 0);
        bool ok2 = active_mode == DDSM_POSITION || command_motor(2, mode_name, 0);
        reply(ok1 && ok2 ? "OK stop" : "ERR stop"); return;
    }
    if (strcmp(command, "disable") == 0) {
        ddsm_frame frame; ddsm_feedback feedback;
        bool ok = ddsm_mode(frame, 1, DDSM_DISABLE) && exchange(frame, DDSM_SWITCH_REPLY, &feedback) &&
                  ddsm_mode(frame, 2, DDSM_DISABLE) && exchange(frame, DDSM_SWITCH_REPLY, &feedback);
        reply(ok ? "OK disabled" : "ERR disable"); return;
    }
    reply("ERR unknown; send HELP");
}

#ifdef DDSM_MOBILE_CONTROL
void app_main(void)
{
    uart_config_t motor_config = {.baud_rate=DDSM_BAUD,.data_bits=UART_DATA_8_BITS,.parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    uart_config_t usb_config = motor_config; usb_config.baud_rate = 115200;
    uart_driver_install(MOTOR_UART, 512, 0, 0, NULL, 0); uart_param_config(MOTOR_UART, &motor_config);
    uart_set_pin(MOTOR_UART, MOTOR_TX, MOTOR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(USB_UART, 2048, 0, 0, NULL, 0); uart_param_config(USB_UART, &usb_config);
    bool enabled1 = set_mode(1, DDSM_OPEN);
    bool enabled2 = set_mode(2, DDSM_OPEN);
    motors_ready = enabled1 && enabled2;
    active_mode = DDSM_OPEN;
    if (!enabled1 || !enabled2) reply("ERR enable; check motor IDs and wiring");
    reply("DDSM arrow control ready; U/D/L/R/S at 115200");
    char line[96]; size_t length = 0;
    int64_t next_feedback = esp_timer_get_time() + 250000;
    for (;;) {
        uint8_t byte;
        if (uart_read_bytes(USB_UART, &byte, 1, pdMS_TO_TICKS(20)) == 1) {
            if (byte == '\r' || byte == '\n') { line[length] = 0; process_line(line); length = 0; }
            else if (length + 1 < sizeof(line) && isprint(byte)) line[length++] = (char)byte;
        }
        if (esp_timer_get_time() >= next_feedback) {
            ddsm_feedback id1_feedback, id2_feedback;
            bool ok1 = report_motor(1, commanded_id1, &id1_feedback);
            bool ok2 = report_motor(2, commanded_id2, &id2_feedback);
            if (active_motion == 'U' || active_motion == 'D') {
                int direction = active_motion == 'U' ? 1 : -1;
                if (!ok1 || !ok2 ||
                    !wheel_sync_update(&mobile_sync,
                                       direction * id2_feedback.speed_x10 / 10.0f,
                                       -direction * id1_feedback.speed_x10 / 10.0f)) {
                    (void)open_command_pair('S');
                    reply("ERR wheel feedback; stopped");
                } else {
                    update_pair_targets();
                }
            }
            next_feedback = esp_timer_get_time() + 250000;
        }
    }
}
#endif
