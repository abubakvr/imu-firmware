#include "step_detect.h"

#include "config.h"

#include <math.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_lock;
static float s_avg_mag = 1.0f;
static float s_prev_hp;
static bool s_peak_armed;
static bool s_have_cadence;
static int64_t s_last_step_us;
static uint32_t s_step_count;
static uint8_t s_session_steps;
static bool s_walking;

static bool step_interval_valid(float interval_s)
{
    return interval_s >= STEP_MIN_INTERVAL_S && interval_s <= STEP_MAX_INTERVAL_S;
}

static void reset_session_locked(void)
{
    s_walking = false;
    s_session_steps = 0;
    s_step_count = 0;
    s_have_cadence = false;
    s_last_step_us = 0;
    s_peak_armed = false;
}

static void refresh_walking_state(int64_t now_us)
{
    if (s_last_step_us == 0) {
        s_walking = false;
        return;
    }

    float idle_s = (float)(now_us - s_last_step_us) * 1e-6f;
    if (idle_s > WALKING_IDLE_TIMEOUT_S) {
        reset_session_locked();
    }
}

void step_detect_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_avg_mag = 1.0f;
    s_prev_hp = 0.0f;
    s_peak_armed = false;
    s_have_cadence = false;
    s_last_step_us = 0;
    s_step_count = 0;
    s_session_steps = 0;
    s_walking = false;
}

void step_detect_update(float ax_g, float ay_g, float az_g, float gyro_mag_dps)
{
    float mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    const float alpha = 0.035f;
    s_avg_mag += alpha * (mag - s_avg_mag);
    float hp = mag - s_avg_mag;

    int64_t now_us = esp_timer_get_time();
    bool step = false;

    if (hp > STEP_PEAK_THRESH_G && hp >= s_prev_hp) {
        s_peak_armed = true;
    }

    if (s_peak_armed && hp < s_prev_hp && s_prev_hp >= STEP_MIN_PEAK_G &&
        gyro_mag_dps >= STEP_MIN_GYRO_DPS) {
        if (!s_have_cadence) {
            s_have_cadence = true;
            s_last_step_us = now_us;
        } else {
            float interval_s = (float)(now_us - s_last_step_us) * 1e-6f;
            if (step_interval_valid(interval_s)) {
                step = true;
            } else if (interval_s > STEP_MAX_INTERVAL_S) {
                s_last_step_us = now_us;
                s_session_steps = 0;
                s_walking = false;
            }
        }
        s_peak_armed = false;
    }

    s_prev_hp = hp;

    if (!step) {
        if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
            refresh_walking_state(now_us);
            xSemaphoreGive(s_lock);
        }
        return;
    }

    if (xSemaphoreTake(s_lock, portMAX_DELAY) != pdTRUE) {
        return;
    }

    s_step_count++;
    s_session_steps++;
    s_last_step_us = now_us;
    if (s_session_steps >= (uint8_t)WALKING_MIN_STEPS) {
        s_walking = true;
    }
    refresh_walking_state(now_us);
    xSemaphoreGive(s_lock);
}

bool step_detect_is_walking(void)
{
    bool walking = false;

    if (!s_lock) {
        return false;
    }
    if (xSemaphoreTake(s_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    refresh_walking_state(esp_timer_get_time());
    walking = s_walking;
    xSemaphoreGive(s_lock);
    return walking;
}

uint32_t step_detect_get_step_count(void)
{
    uint32_t count = 0;

    if (!s_lock) {
        return 0;
    }
    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        count = s_step_count;
        xSemaphoreGive(s_lock);
    }
    return count;
}
