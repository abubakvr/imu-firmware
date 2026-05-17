#include "imu_fusion.h"

#include "config.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define BETA MADGWICK_BETA

static SemaphoreHandle_t s_lock;
static float s_q0 = 1.0f;
static float s_q1 = 0.0f;
static float s_q2 = 0.0f;
static float s_q3 = 0.0f;

static float s_ref0 = 1.0f;
static float s_ref1 = 0.0f;
static float s_ref2 = 0.0f;
static float s_ref3 = 0.0f;

static float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void quat_normalize(float *q0, float *q1, float *q2, float *q3)
{
    float norm = inv_sqrt((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
    *q0 *= norm;
    *q1 *= norm;
    *q2 *= norm;
    *q3 *= norm;
}

void imu_fusion_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_q0 = 1.0f;
    s_q1 = 0.0f;
    s_q2 = 0.0f;
    s_q3 = 0.0f;
    s_ref0 = 1.0f;
    s_ref1 = 0.0f;
    s_ref2 = 0.0f;
    s_ref3 = 0.0f;
}

void imu_fusion_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float q0 = s_q0;
    float q1 = s_q1;
    float q2 = s_q2;
    float q3 = s_q3;

    float recip_norm;
    float s0, s1, s2, s3;
    float q_dot1, q_dot2, q_dot3, q_dot4;

    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        return;
    }

    recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    s0 = q0 * q2 - q1 * q3;
    s1 = q1 * q2 + q0 * q3;
    s2 = 0.5f - q1 * q1 - q2 * q2;
    s3 = q0 * q1 - q2 * q3;

    q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - BETA * s0;
    q_dot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - BETA * s1;
    q_dot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - BETA * s2;
    q_dot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - BETA * s3;

    q0 += q_dot1 * dt;
    q1 += q_dot2 * dt;
    q2 += q_dot3 * dt;
    q3 += q_dot4 * dt;

    quat_normalize(&q0, &q1, &q2, &q3);

    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        s_q0 = q0;
        s_q1 = q1;
        s_q2 = q2;
        s_q3 = q3;
        xSemaphoreGive(s_lock);
    }
}

void imu_fusion_reset_reference(void)
{
    if (xSemaphoreTake(s_lock, portMAX_DELAY) != pdTRUE) {
        return;
    }
    s_ref0 = s_q0;
    s_ref1 = s_q1;
    s_ref2 = s_q2;
    s_ref3 = s_q3;
    xSemaphoreGive(s_lock);
}

void imu_fusion_get_display_quat(float *w, float *x, float *y, float *z)
{
    float q0, q1, q2, q3;
    float r0, r1, r2, r3;

    if (!s_lock) {
        *w = 1.0f;
        *x = 0.0f;
        *y = 0.0f;
        *z = 0.0f;
        return;
    }

    if (xSemaphoreTake(s_lock, portMAX_DELAY) != pdTRUE) {
        return;
    }
    q0 = s_q0;
    q1 = s_q1;
    q2 = s_q2;
    q3 = s_q3;
    r0 = s_ref0;
    r1 = s_ref1;
    r2 = s_ref2;
    r3 = s_ref3;
    xSemaphoreGive(s_lock);

    float inv_r0 = r0;
    float inv_r1 = -r1;
    float inv_r2 = -r2;
    float inv_r3 = -r3;

    *w = inv_r0 * q0 - inv_r1 * q1 - inv_r2 * q2 - inv_r3 * q3;
    *x = inv_r0 * q1 + inv_r1 * q0 + inv_r2 * q3 - inv_r3 * q2;
    *y = inv_r0 * q2 - inv_r1 * q3 + inv_r2 * q0 + inv_r3 * q1;
    *z = inv_r0 * q3 + inv_r1 * q2 - inv_r2 * q1 + inv_r3 * q0;

    quat_normalize(w, x, y, z);
}
