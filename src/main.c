#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "app_status.h"
#include "config.h"
#include "health_sensors.h"
#include "icm20948.h"
#include "mq135.h"
#include "imu_fusion.h"
#include "sensor_status.h"
#include "nvs_flash.h"
#include "oled_display.h"
#include "web_server.h"

#define WIFI_SSID "MTN_4G_570F46"
#ifndef WIFI_PASS
#define WIFI_PASS "74BCA25B"
#endif
#define WIFI_MAX_RETRY 10
#define WIFI_CONNECT_TIMEOUT_MS 60000

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "main";
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num;

static const char *disconnect_reason_str(wifi_err_reason_t reason)
{
    switch (reason) {
    case WIFI_REASON_NO_AP_FOUND: return "AP not found";
    case WIFI_REASON_AUTH_FAIL: return "auth failed";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "wrong password?";
    default: return "disconnect";
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi disconnected: %d (%s)", event->reason,
                 disconnect_reason_str(event->reason));

        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            app_status_set_wifi_connected(false);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip[16];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "got IP: %s", ip);
        app_status_set_ip(ip);
        app_status_set_wifi_connected(true);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "connecting to \"%s\"...", WIFI_SSID);
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    return (bits & WIFI_CONNECTED_BIT) != 0;
}

static void log_open_url(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        return;
    }
    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        ESP_LOGI(TAG, "Open in browser: http://" IPSTR "/", IP2STR(&ip.ip));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    app_status_init();
    sensor_status_init();

#if MQ_GAS_ENABLE
    if (mq135_init() == ESP_OK) {
        mq135_start_task();
    } else {
        ESP_LOGW(TAG, "MQ gas sensors init failed — check AO pins "
                 "MQ135=%d MQ136=%d MQ4=%d MQ7=%d",
                 MQ135_ADC_GPIO, MQ136_ADC_GPIO, MQ4_ADC_GPIO, MQ7_ADC_GPIO);
    }
#endif

#if OLED_ENABLE
    if (oled_display_init() == ESP_OK) {
        oled_display_show_boot("IMU tracker", "Starting...");
        oled_display_start_task();
    } else {
        ESP_LOGW(TAG, "OLED init failed — check SPI wiring (MOSI=23 CLK=18)");
    }
#endif

    imu_fusion_init();

    bool imu_ok = (icm20948_init() == ESP_OK);
    app_status_set_imu_ok(imu_ok);
    if (imu_ok) {
        ESP_ERROR_CHECK(icm20948_start_task());
#if HEALTH_SENSORS_ENABLE
        i2c_master_bus_handle_t bus = icm20948_get_i2c_bus();
        if (health_sensors_init(bus) == ESP_OK) {
            health_sensors_start_task();
        } else {
            ESP_LOGW(TAG, "health sensors not found (MAX30102 0x57, CJMCU-30205)");
        }
#endif
    } else {
        ESP_LOGE(TAG, "ICM20948 init failed — web UI will load but box will not move");
        ESP_LOGE(TAG, "use SDA=GPIO%d SCL=GPIO%d, NCS→3.3V, AD0→GND for 0x68",
                 ICM20948_SDA_GPIO, ICM20948_SCL_GPIO);
    }

#if OLED_ENABLE
    oled_display_show_boot("WiFi", "Connecting...");
#endif

    if (!wifi_init_sta()) {
        ESP_LOGE(TAG, "WiFi failed — IMU viewer needs network");
        app_status_set_wifi_connected(false);
        return;
    }

    app_status_set_wifi_connected(true);
    {
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif) {
            esp_netif_ip_info_t ip;
            if (esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
                char ip_str[16];
                snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip.ip));
                app_status_set_ip(ip_str);
            }
        }
    }

    log_open_url();

    if (web_server_start() != ESP_OK) {
        ESP_LOGE(TAG, "web server failed");
        return;
    }

    ESP_LOGI(TAG, "IMU viewer running");
}
