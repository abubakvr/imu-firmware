#pragma once

#include <stdbool.h>

void app_status_init(void);
void app_status_set_wifi_connected(bool connected);
void app_status_set_ip(const char *ip);
bool app_status_wifi_connected(void);
const char *app_status_ip(void);
void app_status_set_imu_ok(bool ok);
bool app_status_imu_ok(void);
