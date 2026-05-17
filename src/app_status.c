#include "app_status.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_lock;
static bool s_wifi_connected;
static bool s_imu_ok;
static char s_ip[16];

void app_status_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_wifi_connected = false;
    s_imu_ok = false;
    s_ip[0] = '\0';
}

void app_status_set_wifi_connected(bool connected)
{
    if (!s_lock) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_wifi_connected = connected;
    if (!connected) {
        s_ip[0] = '\0';
    }
    xSemaphoreGive(s_lock);
}

void app_status_set_ip(const char *ip)
{
    if (!s_lock) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (ip) {
        strncpy(s_ip, ip, sizeof(s_ip) - 1);
        s_ip[sizeof(s_ip) - 1] = '\0';
    } else {
        s_ip[0] = '\0';
    }
    xSemaphoreGive(s_lock);
}

bool app_status_wifi_connected(void)
{
    bool connected = false;
    if (!s_lock) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    connected = s_wifi_connected;
    xSemaphoreGive(s_lock);
    return connected;
}

const char *app_status_ip(void)
{
    return s_ip;
}

void app_status_set_imu_ok(bool ok)
{
    if (!s_lock) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_imu_ok = ok;
    xSemaphoreGive(s_lock);
}

bool app_status_imu_ok(void)
{
    bool ok = false;
    if (!s_lock) {
        return false;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    ok = s_imu_ok;
    xSemaphoreGive(s_lock);
    return ok;
}
