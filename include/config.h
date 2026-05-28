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
/* IMU fusion loop period (5 ms ≈ 200 Hz). */
#ifndef ICM20948_FUSION_INTERVAL_MS
#define ICM20948_FUSION_INTERVAL_MS 5
#endif
#ifndef ICM20948_LOG_INTERVAL_MS
#define ICM20948_LOG_INTERVAL_MS ICM20948_FUSION_INTERVAL_MS
#endif
#ifndef ICM20948_SERIAL_LOG_MS
#define ICM20948_SERIAL_LOG_MS 500
#endif
#ifndef MADGWICK_BETA
#define MADGWICK_BETA 0.08f
#endif
#ifndef MADGWICK_BETA_MIN
#define MADGWICK_BETA_MIN 0.0f
#endif
/* Accel magnitude within this band of 1 g → trust gravity for tilt correction. */
#ifndef ACCEL_TRUST_BAND_G
#define ACCEL_TRUST_BAND_G 0.12f
#endif
/* Reduce accel trust when gyro rate exceeds this (deg/s). */
#ifndef ACCEL_GYRO_GATE_DPS
#define ACCEL_GYRO_GATE_DPS 25.0f
#endif
#ifndef GYRO_BIAS_SAMPLES
#define GYRO_BIAS_SAMPLES 120
#endif
/* Slow gyro bias tracking while stationary (IIR coefficient per fusion step). */
#ifndef GYRO_BIAS_ONLINE_ALPHA
#define GYRO_BIAS_ONLINE_ALPHA 0.0015f
#endif
#ifndef ACCEL_STILL_BAND_G
#define ACCEL_STILL_BAND_G 0.06f
#endif
/* Hysteresis for UI still flag (gyro magnitude). */
#ifndef GYRO_STILL_ON_DPS
#define GYRO_STILL_ON_DPS 2.5f
#endif
#ifndef GYRO_STILL_OFF_DPS
#define GYRO_STILL_OFF_DPS 4.5f
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
#define OLED_ENABLE 0
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

/* ========== Health sensors (shared I2C with ICM-20948) ========== */
#ifndef HEALTH_SENSORS_ENABLE
#define HEALTH_SENSORS_ENABLE 1
#endif
#ifndef MAX30102_I2C_ADDR
#define MAX30102_I2C_ADDR 0x57
#endif
/* CJMCU-30205 / MAX30205 on shared I2C (SDA=GPIO22 SCL=GPIO21).
 * Address 0x48 = A0/A1/A2 tied to GND; 0x4F = all floating/high (your board). */
#ifndef TEMP_SENSOR_I2C_ADDR
#define TEMP_SENSOR_I2C_ADDR 0x4F
#endif
#ifndef HEALTH_TEMP_RETRY_MS
#define HEALTH_TEMP_RETRY_MS 30000
#endif

/* ========== MQ gas sensors (analog AO on ADC1) ========== */
#ifndef MQ_GAS_ENABLE
#define MQ_GAS_ENABLE 1
#endif
/* Back-compat alias */
#ifndef MQ135_ENABLE
#define MQ135_ENABLE MQ_GAS_ENABLE
#endif
/* ESP32 ADC1: GPIO32–39. One AO pin per sensor. */
#ifndef MQ135_ADC_GPIO
#define MQ135_ADC_GPIO 34
#endif
#ifndef MQ136_ADC_GPIO
#define MQ136_ADC_GPIO 35
#endif
#ifndef MQ4_ADC_GPIO
#define MQ4_ADC_GPIO 32
#endif
#ifndef MQ7_ADC_GPIO
#define MQ7_ADC_GPIO 33
#endif
#ifndef MQ_GAS_SENSOR_COUNT
#define MQ_GAS_SENSOR_COUNT 4
#endif
#ifndef MQ_GAS_POLL_MS
#define MQ_GAS_POLL_MS 500
#endif
#ifndef MQ135_POLL_MS
#define MQ135_POLL_MS MQ_GAS_POLL_MS
#endif
#ifndef MQ_GAS_ADC_SAMPLES
#define MQ_GAS_ADC_SAMPLES 8
#endif
#ifndef MQ135_ADC_SAMPLES
#define MQ135_ADC_SAMPLES MQ_GAS_ADC_SAMPLES
#endif
