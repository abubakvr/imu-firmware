#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

void health_sensors_i2c_diagnose(i2c_master_bus_handle_t bus);
esp_err_t health_sensors_init(i2c_master_bus_handle_t bus);
esp_err_t health_sensors_start_task(void);
