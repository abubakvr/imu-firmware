#pragma once

#include <stdbool.h>
#include <stdint.h>

void step_detect_init(void);
void step_detect_update(float ax_g, float ay_g, float az_g, float gyro_mag_dps);

bool step_detect_is_walking(void);
uint32_t step_detect_get_step_count(void);
