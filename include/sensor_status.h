#pragma once

#include <stdbool.h>

#include "config.h"

void sensor_status_init(void);

void sensor_status_set_temp(float celsius, bool valid);
void sensor_status_set_vitals(int hr_bpm, int spo2_pct, bool valid);

#if MQ_GAS_ENABLE
void sensor_status_set_mq_gas(int index, int raw, bool valid);
bool sensor_status_mq_gas_valid(int index);
int sensor_status_mq_gas_raw(int index);
#endif

bool sensor_status_temp_valid(void);
float sensor_status_temp_c(void);

bool sensor_status_vitals_valid(void);
int sensor_status_hr_bpm(void);
int sensor_status_spo2_pct(void);

/* Legacy names — MQ-135 is index 0 */
bool sensor_status_mq135_valid(void);
int sensor_status_mq135_raw(void);
