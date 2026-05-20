#include "sensor_status.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_lock;
static float s_temp_c;
static int s_hr_bpm;
static int s_spo2_pct;
static bool s_temp_valid;
static bool s_vitals_valid;
static int s_mq135_raw;
static bool s_mq135_valid;

void sensor_status_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_temp_c = NAN;
    s_hr_bpm = 0;
    s_spo2_pct = 0;
    s_temp_valid = false;
    s_vitals_valid = false;
    s_mq135_raw = 0;
    s_mq135_valid = false;
}

void sensor_status_set_temp(float celsius, bool valid)
{
    if (!s_lock) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_temp_c = celsius;
    s_temp_valid = valid;
    xSemaphoreGive(s_lock);
}

void sensor_status_set_vitals(int hr_bpm, int spo2_pct, bool valid)
{
    if (!s_lock) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_hr_bpm = hr_bpm;
    s_spo2_pct = spo2_pct;
    s_vitals_valid = valid;
    xSemaphoreGive(s_lock);
}

void sensor_status_set_mq135(int raw, bool valid)
{
    if (!s_lock) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_mq135_raw = raw;
    s_mq135_valid = valid;
    xSemaphoreGive(s_lock);
}

bool sensor_status_temp_valid(void)
{
    bool v = false;
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        v = s_temp_valid;
        xSemaphoreGive(s_lock);
    }
    return v;
}

float sensor_status_temp_c(void)
{
    float t = NAN;
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        t = s_temp_c;
        xSemaphoreGive(s_lock);
    }
    return t;
}

bool sensor_status_vitals_valid(void)
{
    bool v = false;
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        v = s_vitals_valid;
        xSemaphoreGive(s_lock);
    }
    return v;
}

int sensor_status_hr_bpm(void)
{
    int v = 0;
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        v = s_hr_bpm;
        xSemaphoreGive(s_lock);
    }
    return v;
}

int sensor_status_spo2_pct(void)
{
    int v = 0;
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        v = s_spo2_pct;
        xSemaphoreGive(s_lock);
    }
    return v;
}

bool sensor_status_mq135_valid(void)
{
    bool v = false;
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        v = s_mq135_valid;
        xSemaphoreGive(s_lock);
    }
    return v;
}

int sensor_status_mq135_raw(void)
{
    int v = 0;
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        v = s_mq135_raw;
        xSemaphoreGive(s_lock);
    }
    return v;
}
