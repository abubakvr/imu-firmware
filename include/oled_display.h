#pragma once

#include "esp_err.h"

esp_err_t oled_display_init(void);
esp_err_t oled_display_start_task(void);
void oled_display_show_boot(const char *line1, const char *line2);
