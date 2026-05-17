#pragma once

/* ========== ICM-20948 (matches working open-rental project) ========== */
#ifndef ICM20948_ENABLE
#define ICM20948_ENABLE 1
#endif
#ifndef ICM20948_I2C_PORT
#define ICM20948_I2C_PORT 0
#endif
#ifndef ICM20948_I2C_ADDR
#define ICM20948_I2C_ADDR 0x68
#endif
/* This board: WHO_AM_I OK with SDA=22 SCL=21 @ 100 kHz (wires swapped vs silkscreen). */
#ifndef ICM20948_SDA_GPIO
#define ICM20948_SDA_GPIO 22
#endif
#ifndef ICM20948_SCL_GPIO
#define ICM20948_SCL_GPIO 21
#endif
#ifndef ICM20948_I2C_HZ
#define ICM20948_I2C_HZ 100000
#endif
#ifndef ICM20948_LOG_INTERVAL_MS
#define ICM20948_LOG_INTERVAL_MS 25
#endif
#ifndef ICM20948_SERIAL_LOG_MS
#define ICM20948_SERIAL_LOG_MS 500
#endif
#ifndef MADGWICK_BETA
#define MADGWICK_BETA 0.12f
#endif
#ifndef GYRO_BIAS_SAMPLES
#define GYRO_BIAS_SAMPLES 80
#endif
/* Below this angular rate (deg/s) after bias removal, orientation is frozen. */
#ifndef GYRO_STATIONARY_DPS
#define GYRO_STATIONARY_DPS 1.5f
#endif

/* Step / walking detection (accel magnitude peaks + gyro gate). */
#ifndef STEP_PEAK_THRESH_G
#define STEP_PEAK_THRESH_G 0.17f
#endif
#ifndef STEP_MIN_PEAK_G
#define STEP_MIN_PEAK_G 0.20f
#endif
#ifndef STEP_MIN_GYRO_DPS
#define STEP_MIN_GYRO_DPS 7.5f
#endif
#ifndef STEP_MIN_INTERVAL_S
#define STEP_MIN_INTERVAL_S 0.32f
#endif
#ifndef STEP_MAX_INTERVAL_S
#define STEP_MAX_INTERVAL_S 2.2f
#endif
#ifndef WALKING_IDLE_TIMEOUT_S
#define WALKING_IDLE_TIMEOUT_S 2.0f
#endif
#ifndef WALKING_MIN_STEPS
#define WALKING_MIN_STEPS 2
#endif

/* ========== 0.96" SPI OLED (SSD1306, 128x64) ========== */
#ifndef OLED_ENABLE
#define OLED_ENABLE 1
#endif
/* VSPI on ESP32: SCK=18 MOSI=23 (use SPI3_HOST in driver code). */
#ifndef OLED_SPI_HOST_ID
#define OLED_SPI_HOST_ID 2
#endif
#ifndef OLED_PIN_MOSI
#define OLED_PIN_MOSI 23
#endif
#ifndef OLED_PIN_CLK
#define OLED_PIN_CLK 18
#endif
#ifndef OLED_PIN_DC
#define OLED_PIN_DC 2
#endif
#ifndef OLED_PIN_CS
#define OLED_PIN_CS 5
#endif
#ifndef OLED_PIN_RST
#define OLED_PIN_RST 4
#endif
#ifndef OLED_WIDTH
#define OLED_WIDTH 128
#endif
#ifndef OLED_HEIGHT
#define OLED_HEIGHT 64
#endif
#ifndef OLED_REFRESH_MS
#define OLED_REFRESH_MS 400
#endif
/* Most 0.96" SPI modules are SH1106; use OLED_CTRL_SSD1306 if yours is SSD1306. */
#ifndef OLED_CTRL_SSD1306
#define OLED_CTRL_SSD1306 0
#endif
#ifndef OLED_CTRL_SH1106
#define OLED_CTRL_SH1106 1
#endif
#ifndef OLED_CONTROLLER
#define OLED_CONTROLLER OLED_CTRL_SH1106
#endif
#ifndef OLED_SPI_HZ
#define OLED_SPI_HZ 4000000
#endif
