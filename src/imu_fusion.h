#pragma once

void imu_fusion_init(void);
void imu_fusion_update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void imu_fusion_get_display_quat(float *w, float *x, float *y, float *z);
void imu_fusion_reset_reference(void);
