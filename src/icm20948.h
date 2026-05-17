#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t icm20948_init(void);
esp_err_t icm20948_start_task(void);
esp_err_t icm20948_read_motion(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                               int16_t *gy, int16_t *gz);
void icm20948_i2c_scan_log(void);
void icm20948_deinit(void);
