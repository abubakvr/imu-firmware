#pragma once

#include <stdbool.h>

void sensor_status_init(void);

void sensor_status_set_temp(float celsius, bool valid);
void sensor_status_set_vitals(int hr_bpm, int spo2_pct, bool valid);
void sensor_status_set_mq135(int raw, bool valid);

bool sensor_status_temp_valid(void);
float sensor_status_temp_c(void);

bool sensor_status_vitals_valid(void);
int sensor_status_hr_bpm(void);
int sensor_status_spo2_pct(void);

bool sensor_status_mq135_valid(void);
int sensor_status_mq135_raw(void);
