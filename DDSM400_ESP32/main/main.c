#include "DDSM400.h"
#include "motor_mode.h"
#include "wheel_sync.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(CONFIG_DDSM_TX) || (CONFIG_DDSM_TX < 0)
#error "Set DDSM motor UART TX GPIO in menuconfig: DDSM400 motor UART"
#endif
#if !defined(CONFIG_DDSM_RX) || (CONFIG_DDSM_RX < 0)
#error "Set DDSM motor UART RX GPIO in menuconfig: DDSM400 motor UART"
#endif

static const char *TAG = "DDSM400";
static const uart_port_t motor_uart = UART_NUM_2;
/* ID2 is the reference wheel. ID1 is mounted opposite, so its rotation is inverted. */
static const uint8_t motor_id_ref = 2;
static const uint8_t motor_id_mirror = 1;

#define MOTOR_TX_GPIO              ((gpio_num_t)CONFIG_DDSM_TX)
#define MOTOR_RX_GPIO              ((gpio_num_t)CONFIG_DDSM_RX)
#define MOTOR_RESPONSE_TIMEOUT_MS  250U
#define MOTOR_COMMAND_TIMEOUT_MS   5000LL
#define MOTOR_TASK_POLL_MS         10U
#define MOTOR_LIVE_LOG_MS          250U
#define MOTOR_ODOM_POLL_MS         25U
#define MOTOR_ODOM_IDLE_POLL_MS    100U
/* Effective rolling diameters. Adjust each value after a measured straight run. */
#define WHEEL_DIAMETER_ID2_M       0.0745f
#define WHEEL_DIAMETER_ID1_M       0.0745f
#define MOTOR_QUEUE_LENGTH         1U

#define WIFI_AP_SSID               "DDSM400_ESP32"
#define WIFI_AP_PASSWORD           "12345678"
#define WIFI_AP_CHANNEL            1
#define WIFI_AP_MAX_CONN           4

static QueueHandle_t motor_command_queue;
static SemaphoreHandle_t motor_state_mutex;
static httpd_handle_t web_server;

typedef enum {
    MOTOR_CMD_ENABLE = 0,
    MOTOR_CMD_DRIVE,
    MOTOR_CMD_ODOM_ZERO,
    MOTOR_CMD_STOP,
    MOTOR_CMD_DISABLE
} motor_command_type_t;

typedef struct {
    motor_command_type_t type;
    float signed_raw;
    uint8_t motor_mask;
    uint8_t accel_time;
} motor_command_t;

typedef struct {
    bool enabled;
    bool connected;
    bool feedback_valid;
    float target_raw;
    float actual_rpm;
    float actual_id1_rpm;
    float actual_id2_rpm;
    float sync_error_rpm;
    float sync_scale_id1;
    float sync_scale_id2;
    float position_progress_deg;
    float current_a;
    float current_id1_a;
    float current_id2_a;
    uint8_t temperature_c;
    uint8_t faults;
    uint8_t alarm;
    float travelled_m;
    float displacement_m;
    bool odom_valid;
    int32_t odom_id1_laps;
    int32_t odom_id2_laps;
    uint16_t odom_id1_pos;
    uint16_t odom_id2_pos;
    int64_t odom_id1_counts;
    int64_t odom_id2_counts;
    float rpm_counts_per_turn;
    char message[48];
} motor_public_state_t;

typedef struct {
    bool started;
    double rpm_turns;
    double counts_start;
    float last_rpm;
    int64_t last_ms;
} odom_rpm_check_t;

static motor_public_state_t motor_public_state;
static wheel_sync_state wheel_sync = { .id2_scale = 1.0f, .id1_scale = 1.0f };
static int sync_direction;

typedef struct {
    bool origin_valid;
    bool active;
    uint16_t origin_id1;
    uint16_t origin_id2;
    float waypoint_id1_deg;
    float waypoint_id2_deg;
    float target_deg;
    uint8_t motor_mask;
    int64_t step_started_ms;
    int64_t last_poll_ms;
} position_move_t;

static bool enable_both(void)
{
    if (ACTIVE_MOTOR_MODE.enable(motor_id_ref) &&
        ACTIVE_MOTOR_MODE.enable(motor_id_mirror)) {
        return true;
    }
    (void)servo_off(motor_id_ref);
    (void)servo_off(motor_id_mirror);
    return false;
}

static bool stop_both(void)
{
    wheel_sync_reset(&wheel_sync);
    sync_direction = 0;
    bool ok_ref = ACTIVE_MOTOR_MODE.stop(motor_id_ref);
    bool ok_mirror = ACTIVE_MOTOR_MODE.stop(motor_id_mirror);
    return ok_ref && ok_mirror;
}

static bool disable_both(void)
{
    wheel_sync_reset(&wheel_sync);
    sync_direction = 0;
    bool ok_ref = stop_emergency(motor_id_ref);
    bool ok_mirror = stop_emergency(motor_id_mirror);
    return ok_ref && ok_mirror;
}

static bool drive_both(float signed_raw, uint8_t accel_time)
{
    int direction = (signed_raw > 0.0f) - (signed_raw < 0.0f);
    if (direction != sync_direction) {
        wheel_sync_reset(&wheel_sync);
        sync_direction = direction;
    }
    /* ID1 is physically mirrored: vehicle-forward requires opposite motor direction. */
    float id2_target = signed_raw *
                       (ACTIVE_MOTOR_MODE.position_target ? 1.0f : wheel_sync.id2_scale);
    float id1_target = -signed_raw *
                       (ACTIVE_MOTOR_MODE.position_target ? 1.0f : wheel_sync.id1_scale);
    id2_target = fmaxf(-ACTIVE_MOTOR_MODE.max_value,
                       fminf(ACTIVE_MOTOR_MODE.max_value, id2_target));
    id1_target = fmaxf(-ACTIVE_MOTOR_MODE.max_value,
                       fminf(ACTIVE_MOTOR_MODE.max_value, id1_target));
    bool ok_ref = ACTIVE_MOTOR_MODE.drive(motor_id_ref, id2_target, accel_time);
    if (!ok_ref) {
        ESP_LOGE(TAG, "ID2 %s command/feedback failed; disabling both motors", ACTIVE_MOTOR_MODE.name);
        (void)disable_both();
        return false;
    }

    bool ok_mirror = ACTIVE_MOTOR_MODE.drive(motor_id_mirror, id1_target, accel_time);
    if (!ok_mirror) {
        ESP_LOGE(TAG, "ID1 %s command/feedback failed; disabling both motors", ACTIVE_MOTOR_MODE.name);
        (void)disable_both();
        return false;
    }

    if (!ACTIVE_MOTOR_MODE.position_target) {
        ddsm_feedback id2_feedback, id1_feedback;
        if (!get_feedback(motor_id_ref, &id2_feedback) ||
            !get_feedback(motor_id_mirror, &id1_feedback) ||
            !wheel_sync_update(&wheel_sync,
                               direction * (float)id2_feedback.speed_x10 / 10.0f,
                               -direction * (float)id1_feedback.speed_x10 / 10.0f)) {
            ESP_LOGE(TAG, "Wheel speed feedback mismatch; disabling both motors");
            (void)disable_both();
            return false;
        }
        ESP_LOGI(TAG, "SYNC ID2=%.1f rpm ID1=%.1f rpm scale2=%.3f scale1=%.3f",
                 fabsf((float)id2_feedback.speed_x10 / 10.0f),
                 fabsf((float)id1_feedback.speed_x10 / 10.0f),
                 wheel_sync.id2_scale, wheel_sync.id1_scale);
    }
    return true;
}

static bool drive_selected(uint8_t motor_mask, float signed_raw, uint8_t accel_time)
{
    bool ok;
    if (motor_mask == 1U) {
        wheel_sync_reset(&wheel_sync);
        sync_direction = 0;
        ok = ACTIVE_MOTOR_MODE.drive(motor_id_mirror, -signed_raw, accel_time);
    } else if (motor_mask == 2U) {
        wheel_sync_reset(&wheel_sync);
        sync_direction = 0;
        ok = ACTIVE_MOTOR_MODE.drive(motor_id_ref, signed_raw, accel_time);
    } else {
        ok = drive_both(signed_raw, accel_time);
    }

    if (ok) {
        ddsm_feedback feedback;
        if ((motor_mask & 1U) != 0U && get_feedback(motor_id_mirror, &feedback)) {
            ESP_LOGI(TAG, "LIVE ID1 %s=%.2f actual=%.1f rpm current=%.3f A temp=%u C fault=0x%02X",
                     ACTIVE_MOTOR_MODE.name, -signed_raw, (float)feedback.speed_x10 / 10.0f,
                     (float)feedback.current_raw * (4.0f / 32767.0f),
                     (unsigned)feedback.temperature, (unsigned)feedback.faults);
        }
        if ((motor_mask & 2U) != 0U && get_feedback(motor_id_ref, &feedback)) {
            ESP_LOGI(TAG, "LIVE ID2 %s=%.2f actual=%.1f rpm current=%.3f A temp=%u C fault=0x%02X",
                     ACTIVE_MOTOR_MODE.name, signed_raw, (float)feedback.speed_x10 / 10.0f,
                     (float)feedback.current_raw * (4.0f / 32767.0f),
                     (unsigned)feedback.temperature, (unsigned)feedback.faults);
        }
    } else {
        uint8_t faults;
        if ((motor_mask & 1U) != 0U) {
            if (get_fault_flags(motor_id_mirror, &faults)) {
                ESP_LOGE(TAG, "LIVE ID1 failed, last fault=0x%02X", (unsigned)faults);
            } else {
                ESP_LOGE(TAG, "LIVE ID1 failed, fault unavailable (check UART reply)");
            }
        }
        if ((motor_mask & 2U) != 0U) {
            if (get_fault_flags(motor_id_ref, &faults)) {
                ESP_LOGE(TAG, "LIVE ID2 failed, last fault=0x%02X", (unsigned)faults);
            } else {
                ESP_LOGE(TAG, "LIVE ID2 failed, fault unavailable (check UART reply)");
            }
        }
    }
    return ok;
}

static void state_update(const motor_public_state_t *new_state)
{
    if (motor_state_mutex == NULL || new_state == NULL) {
        return;
    }

    xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
    motor_public_state = *new_state;
    xSemaphoreGive(motor_state_mutex);
}

static motor_public_state_t state_snapshot(void)
{
    motor_public_state_t snapshot = {0};

    if (motor_state_mutex != NULL) {
        xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
        snapshot = motor_public_state;
        xSemaphoreGive(motor_state_mutex);
    }

    return snapshot;
}

static void wifi_ap_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(ret);
    }

    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "Cannot create Wi-Fi AP interface");
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .password = WIFI_AP_PASSWORD,
            .ssid_len = 0,
            .channel = WIFI_AP_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = 0,
            .max_connection = WIFI_AP_MAX_CONN,
            .beacon_interval = 100,
            .pmf_cfg = {
                .capable = true,
                .required = false
            }
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP: SSID=%s, password=%s", WIFI_AP_SSID, WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "Open http://192.168.4.1");
}

/*
 * UART request/response transport. The sliding 10-byte window allows the
 * receiver to recover if bytes are received before frame alignment.
 * Only motor_control_task() calls the DDSM API, so UART access is serialized.
 */
static bool exchange(const ddsm_frame tx, uint8_t expected_opcode,
                     ddsm_feedback *feedback)
{
    uint8_t window[DDSM_SIZE];
    size_t count = 0;

    if (tx == NULL || feedback == NULL) {
        return false;
    }

    (void)uart_flush_input(motor_uart);

    int written = uart_write_bytes(motor_uart, (const char *)tx, DDSM_SIZE);
    if (written != DDSM_SIZE) {
        ESP_LOGE(TAG, "UART write failed (%d/%d bytes)", written, DDSM_SIZE);
        return false;
    }

    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(MOTOR_RESPONSE_TIMEOUT_MS);

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
        uint8_t byte = 0;
        int received = uart_read_bytes(motor_uart, &byte, 1, pdMS_TO_TICKS(2));
        if (received != 1) {
            continue;
        }

        if (count == DDSM_SIZE) {
            memmove(window, window + 1, DDSM_SIZE - 1);
            count = DDSM_SIZE - 1;
        }
        window[count++] = byte;

        if (count == DDSM_SIZE &&
            ddsm_decode(window, tx[0], expected_opcode, feedback)) {
            return true;
        }
    }

    ESP_LOGW(TAG, "No valid reply: ID=%u TX opcode=0x%02X, expected RX=0x%02X",
             (unsigned)tx[0], tx[1], expected_opcode);
    return false;
}

static void update_state_from_driver(bool connected, const char *message,
                                     float target_raw)
{
    motor_public_state_t state = {0};
    motor_public_state_t old_state = state_snapshot();
    ddsm_feedback feedback;
    ddsm_feedback feedback_mirror;
    uint8_t faults = 0;

    state.enabled = servo_is_enabled(motor_id_ref) && servo_is_enabled(motor_id_mirror);
    state.connected = connected;
    state.target_raw = target_raw;
    state.position_progress_deg = old_state.position_progress_deg;
    state.sync_scale_id1 = wheel_sync.id1_scale;
    state.sync_scale_id2 = wheel_sync.id2_scale;
    state.travelled_m = old_state.travelled_m;
    state.displacement_m = old_state.displacement_m;
    state.odom_valid = old_state.odom_valid;
    state.odom_id1_laps = old_state.odom_id1_laps;
    state.odom_id2_laps = old_state.odom_id2_laps;
    state.odom_id1_pos = old_state.odom_id1_pos;
    state.odom_id2_pos = old_state.odom_id2_pos;
    state.odom_id1_counts = old_state.odom_id1_counts;
    state.odom_id2_counts = old_state.odom_id2_counts;
    state.rpm_counts_per_turn = old_state.rpm_counts_per_turn;
    state.alarm = GetAlarmType(motor_id_ref);

    if (get_feedback(motor_id_ref, &feedback)) {
        state.feedback_valid = true;
        state.actual_rpm = (float)feedback.speed_x10 / 10.0f;
        state.actual_id2_rpm = state.actual_rpm;
        state.current_a = (float)feedback.current_raw * (4.0f / 32767.0f);
        state.current_id2_a = state.current_a;
        if (get_feedback(motor_id_mirror, &feedback_mirror)) {
            /* ID1 is mounted in reverse; report both wheels in vehicle direction. */
            state.actual_id1_rpm = -(float)feedback_mirror.speed_x10 / 10.0f;
            state.sync_error_rpm = state.actual_id2_rpm - state.actual_id1_rpm;
            state.current_id1_a = (float)feedback_mirror.current_raw *
                                  (4.0f / 32767.0f);
            if (fabsf(state.current_id1_a) > fabsf(state.current_a)) {
                state.current_a = state.current_id1_a;
            }
        }
        state.temperature_c = feedback.temperature;
        state.faults = feedback.faults;
        state.connected = true;
    } else if (get_fault_flags(motor_id_ref, &faults)) {
        state.faults = faults;
        state.alarm = GetAlarmType(motor_id_ref);
    }

    if (message != NULL) {
        (void)snprintf(state.message, sizeof(state.message), "%s", message);
    }

    state_update(&state);
}

static void update_distance_display(float travelled_m, float displacement_m)
{
    if (motor_state_mutex == NULL) {
        return;
    }
    xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
    motor_public_state.travelled_m = travelled_m;
    motor_public_state.displacement_m = displacement_m;
    xSemaphoreGive(motor_state_mutex);
}

static void update_odom_raw(const ddsm_distance_state state[2])
{
    if (motor_state_mutex == NULL || state == NULL) {
        return;
    }

    xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
    motor_public_state.odom_valid = state[0].initialized && state[1].initialized;
    motor_public_state.odom_id2_laps = state[0].last_turns;
    motor_public_state.odom_id2_pos = state[0].last_position;
    motor_public_state.odom_id2_counts = distance_displacement_counts(&state[0]);
    motor_public_state.odom_id1_laps = state[1].last_turns;
    motor_public_state.odom_id1_pos = state[1].last_position;
    motor_public_state.odom_id1_counts = distance_displacement_counts(&state[1]);
    xSemaphoreGive(motor_state_mutex);
}

static bool reset_distance_measurement(ddsm_distance_state state[2])
{
    ddsm_distance_state next[2] = {0};
    float travelled_id2_m = 0.0f;
    float displacement_id2_m = 0.0f;
    float travelled_id1_m = 0.0f;
    float displacement_id1_m = 0.0f;

    if (state == NULL ||
        !get_distance(motor_id_ref, WHEEL_DIAMETER_ID2_M, &next[0],
                      &travelled_id2_m, &displacement_id2_m, NULL) ||
        !get_distance(motor_id_mirror, WHEEL_DIAMETER_ID1_M, &next[1],
                      &travelled_id1_m, &displacement_id1_m, NULL)) {
        return false;
    }

    state[0] = next[0];
    state[1] = next[1];
    update_distance_display(0.0f, 0.0f);
    update_odom_raw(state);
    return true;
}

static bool update_distance_measurement(ddsm_distance_state state[2])
{
    if (state == NULL) {
        return false;
    }

    ddsm_distance_state next[2] = {state[0], state[1]};
    float travelled_id2_m = 0.0f;
    float displacement_id2_m = 0.0f;
    float travelled_id1_m = 0.0f;
    float displacement_id1_native_m = 0.0f;

    if (!get_distance(motor_id_ref, WHEEL_DIAMETER_ID2_M, &next[0],
                      &travelled_id2_m, &displacement_id2_m, NULL) ||
        !get_distance(motor_id_mirror, WHEEL_DIAMETER_ID1_M, &next[1],
                      &travelled_id1_m, &displacement_id1_native_m, NULL)) {
        return false;
    }

    state[0] = next[0];
    state[1] = next[1];

    /* ID1 is mounted in reverse. Travel is cumulative and unsigned; signed
     * displacement is reported in the vehicle-forward coordinate system. */
    float travelled_m = (travelled_id2_m + travelled_id1_m) * 0.5f;
    float displacement_m =
        (displacement_id2_m - displacement_id1_native_m) * 0.5f;
    update_distance_display(travelled_m, displacement_m);
    update_odom_raw(state);
    return true;
}

static void update_rpm_odometry_check(odom_rpm_check_t *check,
                                      const ddsm_distance_state state[2],
                                      int64_t now_ms)
{
    ddsm_feedback id2;
    ddsm_feedback id1;
    if (check == NULL || state == NULL ||
        !get_feedback(motor_id_ref, &id2) ||
        !get_feedback(motor_id_mirror, &id1)) {
        return;
    }

    float rpm = 0.05f *
        (fabsf((float)id2.speed_x10) + fabsf((float)id1.speed_x10));
    double counts = 0.5 *
        ((double)state[0].travelled_counts + (double)state[1].travelled_counts);

    if (!check->started) {
        if (rpm < 5.0f) {
            return;
        }
        check->started = true;
        check->counts_start = counts;
        check->last_rpm = rpm;
        check->last_ms = now_ms;
        return;
    }

    double dt_min = (double)(now_ms - check->last_ms) / 60000.0;
    if (dt_min <= 0.0) {
        return;
    }
    check->rpm_turns += 0.5 * (double)(rpm + check->last_rpm) * dt_min;
    check->last_rpm = rpm;
    check->last_ms = now_ms;

    if (check->rpm_turns >= 0.25 && motor_state_mutex != NULL) {
        float counts_per_turn =
            (float)((counts - check->counts_start) / check->rpm_turns);
        xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
        motor_public_state.rpm_counts_per_turn = counts_per_turn;
        xSemaphoreGive(motor_state_mutex);
    }
}

static void update_position_progress(float wheel_degrees)
{
    if (motor_state_mutex == NULL) return;
    xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
    motor_public_state.position_progress_deg = wheel_degrees;
    xSemaphoreGive(motor_state_mutex);
}

static bool position_capture_origin(position_move_t *move)
{
    if (!get_position_raw(motor_id_ref, &move->origin_id2) ||
        !get_position_raw(motor_id_mirror, &move->origin_id1)) {
        return false;
    }
    move->origin_valid = true;
    move->waypoint_id1_deg = 0.0f;
    move->waypoint_id2_deg = 0.0f;
    update_position_progress(0.0f);
    return true;
}

static bool position_send_next(position_move_t *move, int64_t now_ms)
{
    (void)move;
    (void)now_ms;
    return true;
}

static bool position_poll(position_move_t *move, int64_t now_ms)
{
    (void)move;
    (void)now_ms;
    return true;
}

static bool position_begin(position_move_t *move, const motor_command_t *command,
                           bool *motor_enabled)
{
    if (!*motor_enabled) {
        *motor_enabled = enable_both();
    }
    if (!*motor_enabled) {
        (void)disable_both();
        *motor_enabled = false;
        update_state_from_driver(false, "Position enable failed", 0.0f);
        return false;
    }

    move->target_deg = command->signed_raw;
    move->motor_mask = command->motor_mask;
    bool ok = true;
    if ((command->motor_mask & 2U) != 0U) {
        ok = ACTIVE_MOTOR_MODE.drive(motor_id_ref, command->signed_raw, 0);
    }
    if (ok && (command->motor_mask & 1U) != 0U) {
        ok = ACTIVE_MOTOR_MODE.drive(motor_id_mirror, -command->signed_raw, 0);
    }
    if (!ok) {
        (void)disable_both();
        *motor_enabled = false;
        update_state_from_driver(false, "Position command failed", 0.0f);
        return false;
    }
    move->active = false;
    update_position_progress(command->signed_raw);
    update_state_from_driver(true, "Position target sent", command->signed_raw);
    return true;
}

static void handle_motor_command(const motor_command_t *command,
                                 bool *motor_enabled,
                                 int64_t *last_drive_ms,
                                 ddsm_distance_state *distance_state)
{
    if (command == NULL || motor_enabled == NULL || last_drive_ms == NULL ||
        distance_state == NULL) {
        return;
    }

    switch (command->type) {
    case MOTOR_CMD_ENABLE:
        if (!*motor_enabled) {
            *motor_enabled = enable_both();
        }
        update_state_from_driver(*motor_enabled,
                                 *motor_enabled ? "Motor enabled" : "Enable failed",
                                 0.0f);
        ESP_LOGI(TAG, "Enable request: %s", *motor_enabled ? "OK" : "FAILED");
        break;

    case MOTOR_CMD_DRIVE: {
        if (!*motor_enabled) {
            *motor_enabled = enable_both();
        }

        if (!*motor_enabled) {
            (void)disable_both();
            *last_drive_ms = 0;
            update_state_from_driver(false, "Enable failed", 0.0f);
            break;
        }

        bool ok = drive_selected(command->motor_mask, command->signed_raw,
                                 command->accel_time);

        if (!ok) {
            /* Communication failure or reported motor fault: best-effort disable. */
            (void)disable_both();
            *motor_enabled = false;
            *last_drive_ms = 0;
            update_state_from_driver(false, "Drive failed; disable requested", 0.0f);
            ESP_LOGE(TAG, "Drive command failed; motor disable requested");
            break;
        }

        *motor_enabled = true;
        *last_drive_ms = esp_timer_get_time() / 1000LL;
        update_state_from_driver(true, "Driving", command->signed_raw);
        break;
    }

    case MOTOR_CMD_ODOM_ZERO:
        if (reset_distance_measurement(distance_state)) {
            if (motor_state_mutex != NULL) {
                xSemaphoreTake(motor_state_mutex, portMAX_DELAY);
                motor_public_state.rpm_counts_per_turn = 0.0f;
                xSemaphoreGive(motor_state_mutex);
            }
            update_state_from_driver(*motor_enabled, "Odometry zeroed", 0.0f);
        } else {
            update_state_from_driver(false, "Odometry zero failed", 0.0f);
        }
        break;

    case MOTOR_CMD_STOP: {
        *last_drive_ms = 0;
        float held_target = ACTIVE_MOTOR_MODE.position_target ?
                            state_snapshot().target_raw : 0.0f;
        if (*motor_enabled && !stop_both()) {
            (void)disable_both();
            *motor_enabled = false;
            update_state_from_driver(false, "Stop failed; disable requested", 0.0f);
        } else {
            update_state_from_driver(*motor_enabled,
                                     *motor_enabled ? (ACTIVE_MOTOR_MODE.position_target ?
                                                       "Holding position target" :
                                                       "Stopped; motor remains enabled") :
                                                      "Motor already disabled",
                                     held_target);
        }
        ESP_LOGI(TAG, "Stop request processed");
        break;
    }

    case MOTOR_CMD_DISABLE: {
        *last_drive_ms = 0;
        bool ack = disable_both();
        *motor_enabled = false; /* Local lockout even if the acknowledgement is lost. */
        update_state_from_driver(ack, ack ? "Motor disabled" : "Disable not confirmed", 0.0f);
        ESP_LOGW(TAG, "Disable request: %s", ack ? "acknowledged" : "NOT confirmed");
        break;
    }

    default:
        break;
    }
}

static void motor_control_task(void *arg)
{
    (void)arg;
    bool motor_enabled = false;
    int64_t last_drive_ms = 0;
    ddsm_distance_state distance_state[2] = {0};
    odom_rpm_check_t odom_rpm_check = {0};
    int64_t last_odom_ms = 0;
    int64_t last_odom_warning_ms = 0;
    float active_raw = 0.0f;
    uint8_t active_mask = 3U;
    int64_t last_live_log_ms = 0;
    position_move_t position_move = {0};

    for (;;) {
        motor_command_t command;

        if (xQueueReceive(motor_command_queue, &command,
                          pdMS_TO_TICKS(MOTOR_TASK_POLL_MS)) == pdTRUE) {
            if (ACTIVE_MOTOR_MODE.position_target &&
                command.type == MOTOR_CMD_DRIVE) {
                (void)position_begin(&position_move, &command, &motor_enabled);
                active_raw = 0.0f;
                active_mask = command.motor_mask;
                continue;
            }
            if (ACTIVE_MOTOR_MODE.position_target && command.type == MOTOR_CMD_STOP) {
                position_move.active = false;
            }
            if (ACTIVE_MOTOR_MODE.position_target && command.type == MOTOR_CMD_DISABLE) {
                position_move.active = false;
                position_move.origin_valid = false;
            }
            bool repeated_drive = command.type == MOTOR_CMD_DRIVE &&
                                  motor_enabled &&
                                  command.motor_mask == active_mask &&
                                  fabsf(command.signed_raw - active_raw) < 0.01f;
            if (repeated_drive) {
                /* HTTP heartbeat; the periodic diagnostic call reads fresh feedback. */
                last_drive_ms = esp_timer_get_time() / 1000LL;
            } else {
                handle_motor_command(&command, &motor_enabled, &last_drive_ms,
                                     distance_state);
                if (command.type == MOTOR_CMD_ODOM_ZERO) {
                    memset(&odom_rpm_check, 0, sizeof(odom_rpm_check));
                }
                if (ACTIVE_MOTOR_MODE.position_target &&
                    command.type == MOTOR_CMD_STOP && motor_enabled) {
                    float held_waypoint = (position_move.motor_mask & 2U) != 0U ?
                        position_move.waypoint_id2_deg : position_move.waypoint_id1_deg;
                    update_state_from_driver(true, "Holding current waypoint",
                        held_waypoint / DDSM_RAW_TURNS_PER_WHEEL_TURN);
                }
                if (ACTIVE_MOTOR_MODE.position_target &&
                    command.type == MOTOR_CMD_ENABLE && motor_enabled &&
                    !position_move.origin_valid &&
                    !position_capture_origin(&position_move)) {
                    (void)disable_both();
                    motor_enabled = false;
                    update_state_from_driver(false, "Position origin read failed", 0.0f);
                }
                if (command.type == MOTOR_CMD_DRIVE && motor_enabled) {
                    active_raw = command.signed_raw;
                    active_mask = command.motor_mask;
                    last_live_log_ms = esp_timer_get_time() / 1000LL;
                } else if (command.type == MOTOR_CMD_STOP ||
                           command.type == MOTOR_CMD_DISABLE) {
                    active_raw = 0.0f;
                    active_mask = 3U;
                }
            }
        }

        int64_t now_ms = esp_timer_get_time() / 1000LL;
        if (ACTIVE_MOTOR_MODE.position_target && position_move.active &&
            !position_poll(&position_move, now_ms)) {
            ESP_LOGE(TAG, "Position feedback/step timeout; disabling both motors");
            (void)disable_both();
            motor_enabled = false;
            position_move.active = false;
            position_move.origin_valid = false;
            update_state_from_driver(false, "Position step timeout; disabled", 0.0f);
        }
        if (motor_enabled && !ACTIVE_MOTOR_MODE.position_target &&
            fabsf(active_raw) > 0.01f &&
            (now_ms - last_live_log_ms) >= MOTOR_LIVE_LOG_MS) {
            last_live_log_ms = now_ms;
            /* Each 250 ms cycle commands both wheels, reads their measured
             * 0x65 RPM, and updates the next cycle's balancing scales. */
            if (drive_selected(active_mask, active_raw, 0)) {
                update_state_from_driver(true, "Driving", active_raw);
            } else {
                ESP_LOGE(TAG, "Live feedback failed; disabling both motors");
                (void)disable_both();
                motor_enabled = false;
                active_raw = 0.0f;
                update_state_from_driver(false, "Live feedback failed", 0.0f);
            }
        }
        uint32_t odom_interval_ms = motor_enabled ? MOTOR_ODOM_POLL_MS :
                                                   MOTOR_ODOM_IDLE_POLL_MS;
        bool odom_due = (now_ms - last_odom_ms) >= odom_interval_ms;
        if (odom_due) {
            last_odom_ms = now_ms;
            if (update_distance_measurement(distance_state)) {
                if (fabsf(active_raw) > 0.01f) {
                    update_rpm_odometry_check(&odom_rpm_check, distance_state,
                                              now_ms);
                }
            } else if ((now_ms - last_odom_warning_ms) >= 1000) {
                last_odom_warning_ms = now_ms;
                ESP_LOGW(TAG, "Odometry update failed; keeping last distance");
            }
        }

        /* Dead-man watchdog: held-drive clients must refresh the target regularly. */
        /* No communication watchdog: motor remains under explicit Stop/Disable control. */
    }
}

static esp_err_t send_command_to_queue(httpd_req_t *request,
                                       motor_command_type_t type,
                                       float signed_raw,
                                       uint8_t accel_time,
                                       uint8_t motor_mask)
{
    (void)request;
    if (motor_command_queue == NULL) {
        return ESP_FAIL;
    }

    motor_command_t command = {
        .type = type,
        .signed_raw = signed_raw,
        .motor_mask = motor_mask,
        .accel_time = accel_time
    };

    /* A one-element overwrite queue keeps only the newest user command.
     * This prevents delayed stale RPM commands building up under slow UART replies. */
    if (xQueueOverwrite(motor_command_queue, &command) != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static bool parse_float_parameter(httpd_req_t *request, const char *key,
                                  float *out_value);
static esp_err_t send_service_unavailable(httpd_req_t *request,
                                          const char *message);

static esp_err_t send_service_unavailable(httpd_req_t *request,
                                          const char *message)
{
    httpd_resp_set_status(request, "503 Service Unavailable");
    httpd_resp_set_type(request, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(request, message);
}

static bool get_query_parameter(httpd_req_t *request, const char *key,
                                char *value, size_t value_size)
{
    char query[128];

    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK) {
        return false;
    }

    return httpd_query_key_value(query, key, value, value_size) == ESP_OK;
}

static bool parse_float_parameter(httpd_req_t *request, const char *key,
                                  float *out_value)
{
    char value[32];
    char *end = NULL;

    if (out_value == NULL || !get_query_parameter(request, key, value, sizeof(value))) {
        return false;
    }

    errno = 0;
    float parsed = strtof(value, &end);
    if (errno == ERANGE || end == value || *end != '\0' || !isfinite(parsed)) {
        return false;
    }

    *out_value = parsed;
    return true;
}

static esp_err_t queue_simple_command(httpd_req_t *request,
                                      motor_command_type_t type,
                                      const char *success_message)
{
    esp_err_t ret = send_command_to_queue(request, type, 0.0f, 0, 3U);
    if (ret != ESP_OK) {
        return send_service_unavailable(request, "Motor command queue unavailable");
    }

    httpd_resp_set_type(request, "application/json");
    char response[80];
    (void)snprintf(response, sizeof(response), "{\"queued\":true,\"message\":\"%s\"}",
                   success_message);
    return httpd_resp_send(request, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t drive_post_handler(httpd_req_t *request)
{
    float target = 0.0f;
    char motor_value[8];
    uint8_t motor_mask = 3U;

    if (!parse_float_parameter(request, "value", &target) ||
        target < -ACTIVE_MOTOR_MODE.max_value ||
        target > ACTIVE_MOTOR_MODE.max_value ||
        (DDSM_APP_MODE == 0 && truncf(target) != target)) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                            "value is outside the selected mode range");
        return ESP_ERR_INVALID_ARG;
    }

    if (get_query_parameter(request, "motor", motor_value, sizeof(motor_value))) {
        if (strcmp(motor_value, "1") == 0) motor_mask = 1U;
        else if (strcmp(motor_value, "2") == 0) motor_mask = 2U;
        else if (strcmp(motor_value, "both") != 0) {
            httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                                "motor must be 1, 2, or both");
            return ESP_ERR_INVALID_ARG;
        }
    }

    esp_err_t ret = send_command_to_queue(request, MOTOR_CMD_DRIVE, target, 0,
                                          motor_mask);
    if (ret != ESP_OK) {
        return send_service_unavailable(request, "Motor command queue unavailable");
    }

    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, "{\"queued\":true}");
}

static esp_err_t stop_post_handler(httpd_req_t *request)
{
    return queue_simple_command(request, MOTOR_CMD_STOP, "stop queued");
}

static esp_err_t odom_zero_post_handler(httpd_req_t *request)
{
    return queue_simple_command(request, MOTOR_CMD_ODOM_ZERO,
                                "odometry zero queued");
}

static esp_err_t enable_post_handler(httpd_req_t *request)
{
    return queue_simple_command(request, MOTOR_CMD_ENABLE, "enable queued");
}

static esp_err_t disable_post_handler(httpd_req_t *request)
{
    return queue_simple_command(request, MOTOR_CMD_DISABLE, "disable queued");
}

static esp_err_t status_get_handler(httpd_req_t *request)
{
    motor_public_state_t state = state_snapshot();
    /* Keep the large JSON buffer out of the HTTP task's stack. */
    static char response[1600];

    int length = snprintf(response, sizeof(response),
        "{\"enabled\":%s,\"connected\":%s,\"feedback_valid\":%s,"
        "\"mode\":\"%s\",\"unit\":\"%s\",\"note\":\"%s\",\"max_value\":%.2f,\"initial_value\":%.2f,"
        "\"step\":%.2f,\"position_target\":%s,\"target_value\":%.2f,\"actual_rpm\":%.1f,"
        "\"actual_id1_rpm\":%.1f,\"actual_id2_rpm\":%.1f,\"sync_error_rpm\":%.1f,"
        "\"sync_scale_id1\":%.3f,\"sync_scale_id2\":%.3f,"
        "\"position_progress_deg\":%.1f,\"current_a\":%.3f,"
        "\"current_id1_a\":%.3f,\"current_id2_a\":%.3f,"
        "\"temperature_c\":%u,\"faults\":%u,\"alarm\":%u,"
        "\"travelled_m\":%.3f,\"travelled_mm\":%.1f,\"displacement_m\":%.3f,"
        "\"odom_counts_per_wheel_turn\":%.0f,\"odom_laps_per_wheel_turn\":%.3f,"
        "\"rpm_counts_per_turn\":%.0f,"
        "\"odom_valid\":%s,\"odom_id2_laps\":%ld,\"odom_id2_pos\":%u,"
        "\"odom_id2_counts\":%lld,\"odom_id1_laps\":%ld,\"odom_id1_pos\":%u,"
        "\"odom_id1_counts\":%lld,\"odom_id2_wheel_turns\":%.3f,"
        "\"odom_id1_wheel_turns\":%.3f,\"message\":\"%s\"}",
        state.enabled ? "true" : "false",
        state.connected ? "true" : "false",
        state.feedback_valid ? "true" : "false",
        ACTIVE_MOTOR_MODE.name, ACTIVE_MOTOR_MODE.label, ACTIVE_MOTOR_MODE.note,
        ACTIVE_MOTOR_MODE.max_value, ACTIVE_MOTOR_MODE.initial_value,
        ACTIVE_MOTOR_MODE.step, ACTIVE_MOTOR_MODE.position_target ? "true" : "false",
        state.target_raw, state.actual_rpm, state.actual_id1_rpm,
        state.actual_id2_rpm, state.sync_error_rpm,
        state.sync_scale_id1, state.sync_scale_id2,
        state.position_progress_deg, state.current_a,
        state.current_id1_a, state.current_id2_a,
        (unsigned)state.temperature_c, (unsigned)state.faults,
        (unsigned)state.alarm, state.travelled_m, state.travelled_m * 1000.0f,
        state.displacement_m, DDSM_ODOM_COUNTS_PER_WHEEL_TURN,
        DDSM_ODOM_LAPS_PER_WHEEL_TURN, state.rpm_counts_per_turn,
        state.odom_valid ? "true" : "false",
        (long)state.odom_id2_laps, (unsigned)state.odom_id2_pos,
        (long long)state.odom_id2_counts,
        (long)state.odom_id1_laps, (unsigned)state.odom_id1_pos,
        (long long)state.odom_id1_counts,
        (double)state.odom_id2_counts / DDSM_ODOM_COUNTS_PER_WHEEL_TURN,
        (double)state.odom_id1_counts / DDSM_ODOM_COUNTS_PER_WHEEL_TURN,
        state.message);

    if (length < 0 || (size_t)length >= sizeof(response)) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Could not build status response");
        return ESP_FAIL;
    }

    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, response, length);
}

static esp_err_t root_get_handler(httpd_req_t *request)
{
    static const char html[] =
        "<!doctype html><html lang='en'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>DDSM400 motor control</title>"
        "<style>body{font:16px system-ui;max-width:650px;margin:24px auto;padding:0 16px;"
        "background:#f5f6f8;color:#20242a}main{background:white;padding:20px;border-radius:14px;"
        "box-shadow:0 2px 12px #0001}button{padding:14px 18px;margin:5px;border:0;border-radius:9px;"
        "font-weight:600} .drive{background:#dceafe;min-width:130px} .stop{background:#ffe8b5}"
        ".off{background:#ffd9d9}pre{white-space:pre-wrap;background:#f0f2f5;padding:12px;"
        "border-radius:8px}input{width:100%}.note{color:#596273;font-size:14px}</style></head><body>"
        "<main><h1>DDSM400 motor control</h1><p>Motor IDs: 2 + 1 mirrored | <b id='modeText'>Loading mode...</b> | UART 38400 8N1</p>"
        "<label><span id='unitText'>Target</span>: <b id='speedText'>0</b></label>"
        "<input id='speed' type='range' min='0' max='1' step='1' value='0' disabled>"
        "<label>Motor: <select id='motor'><option value='both'>Both motors</option>"
        "<option value='2'>ID2 only</option><option value='1'>ID1 only</option></select></label>"
        "<p><button id='forward' class='drive'>Forward / + angle</button>"
        "<button id='reverse' class='drive'>Reverse / - angle</button></p>"
        "<p><button id='stop' class='stop'>Stop</button>"
        "<button id='disable' class='off'>Disable motor</button></p>"
        "<p class='note' id='modeNote'>Loading mode guidance...</p>"
        "<p class='note'>Open, current and speed commands are refreshed while Run is active. "
        "Position sets one target and holds it. Disable removes motor torque.</p>"
        "<h2>Distance traveled</h2><p><b id='distanceText'>0.0 mm</b></p>"
        "<p><button id='odomZero'>Zero odometry</button></p>"
        "<p class='note' id='odomText'>Waiting for odometry...</p>"
        "<h2>Wheel speeds</h2><p><b id='wheelText'>Waiting for feedback...</b></p>"
        "<p id='positionProgress' style='display:none'>Position: <b id='positionText'>0</b> degrees reached</p>"
        "<h2>Live feedback</h2><pre id='status'>Connecting...</pre></main>"
        "<script>const $=id=>document.getElementById(id);let timer=null,active=false,dir=1,inFlight=false,commandSeq=0,positionTarget=false,configured=false;"
        "$('speed').oninput=()=>{$('speedText').textContent=$('speed').value};"
        "async function post(path){try{await fetch(path,{method:'POST',cache:'no-store'})}catch(e){}}"
        "async function drive(){if(!active||inFlight)return;let value=Number($('speed').value)*dir,seq=commandSeq;"
        "inFlight=true;try{await fetch('/api/drive?value='+value+'&motor='+$('motor').value,{method:'POST',cache:'no-store'})}catch(e){}"
        "finally{inFlight=false;if(seq!==commandSeq&&active)drive()}}"
        "function start(d){if(!configured)return;dir=d;commandSeq++;if(active){drive();return}active=true;drive();if(!positionTarget)timer=setInterval(drive,500)}"
        "function release(){commandSeq++;if(!active)return;active=false;if(timer)clearInterval(timer);timer=null;post('/api/stop')}"
        "['forward','reverse'].forEach((id)=>$(id).onclick=()=>start(id==='forward'?1:-1));"
        "$('stop').onclick=()=>{release();post('/api/stop')};"
        "$('disable').onclick=()=>{release();post('/api/disable')};"
        "$('odomZero').onclick=()=>post('/api/odom_zero');"
        "window.addEventListener('blur',()=>{if(!positionTarget)release()});"
        "document.addEventListener('visibilitychange',()=>{if(document.hidden&&!positionTarget)release()});"
        "window.addEventListener('pagehide',()=>{if(active){active=false;post('/api/disable')}});"
        "async function refresh(){try{let r=await fetch('/api/status',{cache:'no-store'});let s=await r.json();"
        "if(!configured){configured=true;positionTarget=s.position_target;$('speed').max=s.max_value;"
        "$('speed').step=s.step;$('speed').value=s.initial_value;$('speed').disabled=false;"
        "$('speedText').textContent=s.initial_value;$('modeText').textContent=s.mode;"
        "$('unitText').textContent=s.unit;$('modeNote').textContent=s.note;"
        "$('stop').textContent=positionTarget?'Hold current step':'Stop';"
        "if(positionTarget)$('positionProgress').style.display='block';}"
        "$('distanceText').textContent=Number(s.travelled_mm||0).toFixed(1)+' mm ('+Number(s.travelled_m||0).toFixed(3)+' m)';"
        "$('odomText').textContent=s.odom_valid?('ID2 laps '+s.odom_id2_laps+' pos '+s.odom_id2_pos+' counts '+s.odom_id2_counts+' ('+Number(s.odom_id2_wheel_turns).toFixed(3)+' wheel turns) | ID1 counts '+s.odom_id1_counts+' | after 10 manual ID2 turns: '+Math.abs(Math.round(s.odom_id2_counts/10))+' counts/turn, '+Math.abs(Number(s.odom_id2_counts)/10/32768).toFixed(3)+' laps/turn | configured '+Number(s.odom_laps_per_wheel_turn).toFixed(3)+' laps/turn'+(Number(s.rpm_counts_per_turn)>0?' | RPM check '+Number(s.rpm_counts_per_turn).toFixed(0)+' counts/turn':'') ):'No odometry reply';"
        "$('wheelText').textContent='ID1 '+Number(s.actual_id1_rpm||0).toFixed(1)+' RPM (raw x'+Number(s.sync_scale_id1||1).toFixed(2)+') | ID2 '+Number(s.actual_id2_rpm||0).toFixed(1)+' RPM (raw x'+Number(s.sync_scale_id2||1).toFixed(2)+') | difference '+Number(s.sync_error_rpm||0).toFixed(1)+' RPM';"
        "if(positionTarget)$('positionText').textContent=Number(s.position_progress_deg||0).toFixed(1);"
        "$('status').textContent=JSON.stringify(s,null,2)}catch(e){$('status').textContent='Status unavailable'}}"
        "refresh();setInterval(refresh,500);</script></body></html>";

    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

    esp_err_t ret = httpd_start(&web_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    static const httpd_uri_t routes[] = {
        { .uri = "/",           .method = HTTP_GET,  .handler = root_get_handler },
        { .uri = "/api/status",.method = HTTP_GET,  .handler = status_get_handler },
        { .uri = "/api/drive", .method = HTTP_POST, .handler = drive_post_handler },
        { .uri = "/api/odom_zero", .method = HTTP_POST, .handler = odom_zero_post_handler },
        { .uri = "/api/stop",  .method = HTTP_POST, .handler = stop_post_handler },
        { .uri = "/api/enable",.method = HTTP_POST, .handler = enable_post_handler },
        { .uri = "/api/disable",.method = HTTP_POST,.handler = disable_post_handler }
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        ret = httpd_register_uri_handler(web_server, &routes[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Register URI %s failed: %s",
                     routes[i].uri, esp_err_to_name(ret));
            httpd_stop(web_server);
            web_server = NULL;
            return ret;
        }
    }

    ESP_LOGI(TAG, "HTTP control page ready at http://192.168.4.1");
    return ESP_OK;
}

#ifndef DDSM_MOBILE_CONTROL
void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate = DDSM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(motor_uart, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(motor_uart, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(motor_uart, MOTOR_TX_GPIO, MOTOR_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART2: TX=%d RX=%d baud=%d",
             (int)MOTOR_TX_GPIO, (int)MOTOR_RX_GPIO, DDSM_BAUD);

    motor_command_queue = xQueueCreate(MOTOR_QUEUE_LENGTH, sizeof(motor_command_t));
    motor_state_mutex = xSemaphoreCreateMutex();
    if (motor_command_queue == NULL || motor_state_mutex == NULL) {
        ESP_LOGE(TAG, "Cannot allocate motor queue/state mutex");
        return;
    }

    servo_init(exchange);

    /* A reset must not cause the previous test cycle to start moving again. */
    if (!disable_both()) {
        ESP_LOGW(TAG, "Startup DISABLE was not confirmed; check motor link and HAT switch");
    }

    motor_public_state.enabled = false;
    motor_public_state.connected = false;
    (void)snprintf(motor_public_state.message,
                   sizeof(motor_public_state.message), "Ready; motor disabled");

    /* Disable the motor before Wi-Fi startup can delay application initialization. */
    wifi_ap_init();

    BaseType_t task_ok = xTaskCreate(motor_control_task, "motor_control",
                                     4096, NULL, 10, NULL);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Cannot create motor control task");
        return;
    }

    ESP_ERROR_CHECK(web_server_start());
    ESP_LOGI(TAG, "Ready. Motor will not move until a drive command is received.");
}
#endif
