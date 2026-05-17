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
#define STEP_PEAK_THRESH_G 0.20f
#endif
#ifndef STEP_MIN_PEAK_G
#define STEP_MIN_PEAK_G 0.24f
#endif
#ifndef STEP_MIN_GYRO_DPS
#define STEP_MIN_GYRO_DPS 10.0f
#endif
#ifndef STEP_MIN_INTERVAL_S
#define STEP_MIN_INTERVAL_S 0.35f
#endif
#ifndef STEP_MAX_INTERVAL_S
#define STEP_MAX_INTERVAL_S 1.75f
#endif
#ifndef WALKING_IDLE_TIMEOUT_S
#define WALKING_IDLE_TIMEOUT_S 1.6f
#endif
#ifndef WALKING_MIN_STEPS
#define WALKING_MIN_STEPS 3
#endif
