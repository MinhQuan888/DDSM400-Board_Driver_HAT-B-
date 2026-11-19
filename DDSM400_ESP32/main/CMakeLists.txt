idf_component_register(
    SRCS
        "main.c"
        "main_open.c"
        "main_speed.c"
        "main_current.c"
        "main_position.c"
        "main_mobile_control.c"
        "wheel_sync.c"
        "DDSM400.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        driver
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        esp_http_server
        esp_timer
)
