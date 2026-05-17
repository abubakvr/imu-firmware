idf_component_register(
    SRCS
        "main.c"
        "icm20948.c"
        "imu_fusion.c"
        "step_detect.c"
        "web_server.c"
        "index_html.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        driver
        esp_http_server
)
