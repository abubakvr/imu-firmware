#include "icm20948.h"

#include "config.h"
#include "imu_fusion.h"
#include "step_detect.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "ICM20948";

#define REG_BANK_SEL      0x7FU
#define REG_WHO_AM_I      0x00U
#define WHO_AM_I_ICM20948 0xEAU

#define REG_USER_CTRL  0x03U
#define REG_PWR_MGMT_1 0x06U
#define REG_PWR_MGMT_2 0x07U
#define REG_ACCEL_XOUT_H 0x2DU
#define REG_GYRO_CONFIG_1 0x01U /* bank 2 */
#define REG_ACCEL_CONFIG  0x14U /* bank 2 */

#define NVS_NS_IMU_CAL "imu_cal"
#define NVS_KEY_CAL_VER "ver"
#define NVS_IMU_CAL_VER 1

/* Removed USER_CTRL_I2C_IF_DIS — setting bit4 disables the I2C slave
   interface on the ICM-20948 itself, which bricks all subsequent I2C reads.
   Only set this when using SPI. */

#define ACCEL_LSB_PER_G   16384.0f
#define GYRO_LSB_PER_DPS  131.0f
#define DEG_TO_RAD        ((float)M_PI / 180.0f)

#define I2C_XFER_TIMEOUT_MS 50

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool  s_i2c_installed;
static bool  s_imu_ok;
static uint8_t  s_i2c_addr;
static int      s_sda_gpio;
static int      s_scl_gpio;
static uint32_t s_i2c_hz;

static float s_gyro_bias_dps[3];
static float s_accel_offset_g[3];
static float s_accel_scale = 1.0f;
static volatile float s_yaw_rate_dps;
static volatile bool s_imu_still = true;
static bool  s_gyro_calibrated;
static uint32_t s_gyro_cal_count;
static float s_gyro_cal_sum[3];
static float s_accel_cal_sum[3];
static volatile bool s_cal_requested;
static portMUX_TYPE s_cal_mux = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------------------------------------------------ */
static float vec3_mag(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static float fusion_accel_beta(float axg, float ayg, float azg, float gyro_mag_dps)
{
    float amag = vec3_mag(axg, ayg, azg);
    float err = fabsf(amag - 1.0f);
    float beta = MADGWICK_BETA;

    if (err >= ACCEL_TRUST_BAND_G) {
        beta = MADGWICK_BETA_MIN;
    } else {
        beta *= 1.0f - (err / ACCEL_TRUST_BAND_G);
    }

    if (gyro_mag_dps > ACCEL_GYRO_GATE_DPS) {
        float t = (gyro_mag_dps - ACCEL_GYRO_GATE_DPS) / ACCEL_GYRO_GATE_DPS;
        if (t > 1.0f) {
            t = 1.0f;
        }
        beta *= 1.0f - t;
    }

    if (beta < MADGWICK_BETA_MIN) {
        beta = MADGWICK_BETA_MIN;
    }
    return beta;
}

static bool bias_stationary(float gyro_mag_dps, float axg, float ayg, float azg)
{
    float amag = vec3_mag(axg, ayg, azg);
    return (gyro_mag_dps < GYRO_STILL_ON_DPS) &&
           (fabsf(amag - 1.0f) < ACCEL_STILL_BAND_G);
}

static void cal_reset_accumulators(void)
{
    s_gyro_cal_count = 0;
    s_gyro_cal_sum[0] = 0.0f;
    s_gyro_cal_sum[1] = 0.0f;
    s_gyro_cal_sum[2] = 0.0f;
    s_accel_cal_sum[0] = 0.0f;
    s_accel_cal_sum[1] = 0.0f;
    s_accel_cal_sum[2] = 0.0f;
}

static void cal_load_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_IMU_CAL, NVS_READONLY, &h) != ESP_OK) {
        return;
    }

    uint8_t ver = 0;
    if (nvs_get_u8(h, NVS_KEY_CAL_VER, &ver) != ESP_OK || ver != NVS_IMU_CAL_VER) {
        nvs_close(h);
        return;
    }

    size_t len = sizeof(s_gyro_bias_dps);
    if (nvs_get_blob(h, "gbias", s_gyro_bias_dps, &len) == ESP_OK && len == sizeof(s_gyro_bias_dps)) {
        s_gyro_calibrated = true;
    }

    len = sizeof(s_accel_offset_g);
    if (nvs_get_blob(h, "aoff", s_accel_offset_g, &len) != ESP_OK || len != sizeof(s_accel_offset_g)) {
        s_accel_offset_g[0] = 0.0f;
        s_accel_offset_g[1] = 0.0f;
        s_accel_offset_g[2] = 0.0f;
    }

    len = sizeof(s_accel_scale);
    if (nvs_get_blob(h, "ascale", &s_accel_scale, &len) != ESP_OK || len != sizeof(s_accel_scale) ||
        s_accel_scale < 0.5f || s_accel_scale > 2.0f) {
        s_accel_scale = 1.0f;
    }

    nvs_close(h);
    if (s_gyro_calibrated) {
        ESP_LOGI(TAG, "loaded cal: gyro bias %.2f,%.2f,%.2f accel scale %.4f",
                 s_gyro_bias_dps[0], s_gyro_bias_dps[1], s_gyro_bias_dps[2], s_accel_scale);
    }
}

static void cal_save_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_IMU_CAL, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }

    uint8_t ver = NVS_IMU_CAL_VER;
    nvs_set_u8(h, NVS_KEY_CAL_VER, ver);
    nvs_set_blob(h, "gbias", s_gyro_bias_dps, sizeof(s_gyro_bias_dps));
    nvs_set_blob(h, "aoff", s_accel_offset_g, sizeof(s_accel_offset_g));
    nvs_set_blob(h, "ascale", &s_accel_scale, sizeof(s_accel_scale));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "saved IMU calibration to NVS");
}

static void cal_finish_from_accumulators(void)
{
    float n = (float)GYRO_BIAS_SAMPLES;
    s_gyro_bias_dps[0] = s_gyro_cal_sum[0] / n;
    s_gyro_bias_dps[1] = s_gyro_cal_sum[1] / n;
    s_gyro_bias_dps[2] = s_gyro_cal_sum[2] / n;

    float mean_ax = s_accel_cal_sum[0] / n;
    float mean_ay = s_accel_cal_sum[1] / n;
    float mean_az = s_accel_cal_sum[2] / n;
    s_accel_offset_g[0] = mean_ax;
    s_accel_offset_g[1] = mean_ay;
    s_accel_offset_g[2] = mean_az;

    float mag = vec3_mag(mean_ax, mean_ay, mean_az);
    if (mag > 0.5f) {
        s_accel_scale = 1.0f / mag;
    } else {
        s_accel_scale = 1.0f;
    }

    s_gyro_calibrated = true;
    cal_save_nvs();
    imu_fusion_reset_reference();
    ESP_LOGI(TAG, "gyro bias (dps): %.2f, %.2f, %.2f | accel scale %.4f",
             s_gyro_bias_dps[0], s_gyro_bias_dps[1], s_gyro_bias_dps[2], s_accel_scale);
}

static void apply_accel_cal(float *axg, float *ayg, float *azg)
{
    *axg = (*axg - s_accel_offset_g[0]) * s_accel_scale;
    *ayg = (*ayg - s_accel_offset_g[1]) * s_accel_scale;
    *azg = (*azg - s_accel_offset_g[2]) * s_accel_scale;
}

void icm20948_request_calibration(void)
{
    portENTER_CRITICAL(&s_cal_mux);
    s_cal_requested = true;
    portEXIT_CRITICAL(&s_cal_mux);
    s_gyro_calibrated = false;
    cal_reset_accumulators();
    ESP_LOGI(TAG, "calibration requested — hold helmet still");
}

/* ------------------------------------------------------------------ */
static esp_err_t set_user_bank(uint8_t bank)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    uint8_t b = (uint8_t)((bank & 0x03U) << 4);
    uint8_t buf[2] = {REG_BANK_SEL, b};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_XFER_TIMEOUT_MS);
}

static esp_err_t write_reg_u8(uint8_t reg, uint8_t val)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_XFER_TIMEOUT_MS);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, I2C_XFER_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */
static void setup_i2c_gpio(void)
{
    gpio_reset_pin((gpio_num_t)s_sda_gpio);
    gpio_reset_pin((gpio_num_t)s_scl_gpio);
    gpio_set_direction((gpio_num_t)s_sda_gpio, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction((gpio_num_t)s_scl_gpio, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level((gpio_num_t)s_sda_gpio, 1);
    gpio_set_level((gpio_num_t)s_scl_gpio, 1);
}

/* ------------------------------------------------------------------ */
/* Tear down bus+device WITHOUT resetting the config variables.
   The init retry loop changes s_sda_gpio / s_scl_gpio / s_i2c_hz /
   s_i2c_addr before calling this, and needs them preserved. */
static void teardown_i2c(void)
{
    s_imu_ok = false;
    if (s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    s_i2c_installed = false;
}

/* Public deinit resets everything including config. */
void icm20948_deinit(void)
{
    teardown_i2c();
    s_i2c_addr = (uint8_t)ICM20948_I2C_ADDR;
    s_sda_gpio = ICM20948_SDA_GPIO;
    s_scl_gpio = ICM20948_SCL_GPIO;
    s_i2c_hz   = ICM20948_I2C_HZ;
}

/* ------------------------------------------------------------------ */
static esp_err_t ensure_i2c(void)
{
    if (s_i2c_installed) return ESP_OK;

    setup_i2c_gpio();

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port        = (i2c_port_num_t)ICM20948_I2C_PORT,
        .sda_io_num      = (gpio_num_t)s_sda_gpio,
        .scl_io_num      = (gpio_num_t)s_scl_gpio,
        .clk_source      = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority   = 0,
        .trans_queue_depth = 0,
        .flags           = {.enable_internal_pullup = 1},
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = (uint16_t)s_i2c_addr,
        .scl_speed_hz    = s_i2c_hz,
        .scl_wait_us     = 300,
    };

    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return err;
    }

    s_i2c_installed = true;
    ESP_LOGI(TAG, "I2C ready SDA=%d SCL=%d addr=0x%02X @ %lu Hz",
             s_sda_gpio, s_scl_gpio, s_i2c_addr, (unsigned long)s_i2c_hz);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
void icm20948_i2c_scan_log(void)
{
    /* Use the current bus if already up; don't call ensure_i2c() here
       because it may re-create the bus with stale config after a failed
       init sequence. */
    if (!s_bus) return;

    ESP_LOGI(TAG, "probing I2C bus (SDA=%d SCL=%d)...", s_sda_gpio, s_scl_gpio);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_master_probe(s_bus, addr, I2C_XFER_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "  device at 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "no I2C ACK — check 3.3V, GND, NCS→3V3, SDA/SCL wiring");
    }
}

/* ------------------------------------------------------------------ */
static esp_err_t icm20948_init_chip(void)
{
    uint8_t who = 0;
    esp_err_t err = read_regs(REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        return err;
    }
    if (who != WHO_AM_I_ICM20948) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X (expected 0x%02X)", who, WHO_AM_I_ICM20948);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "WHO_AM_I OK (0x%02X) at 0x%02X", who, s_i2c_addr);

    /* Reset */
    err = set_user_bank(0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "bank select failed: %s", esp_err_to_name(err));
    }

    err = write_reg_u8(REG_PWR_MGMT_1, 0x80U); /* device reset */
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Select auto clock, clear sleep */
    err = write_reg_u8(REG_PWR_MGMT_1, 0x01U);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Enable accel + gyro */
    err = write_reg_u8(REG_PWR_MGMT_2, 0x00U);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ±250 dps gyro, ±2 g accel (matches ACCEL_LSB_PER_G / GYRO_LSB_PER_DPS). */
    err = set_user_bank(2);
    if (err != ESP_OK) return err;
    err = write_reg_u8(REG_GYRO_CONFIG_1, 0x00U);
    if (err != ESP_OK) return err;
    err = write_reg_u8(REG_ACCEL_CONFIG, 0x00U);
    if (err != ESP_OK) return err;
    err = set_user_bank(0);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Do NOT touch USER_CTRL bit4 (I2C_IF_DIS) — that disables the
       ICM-20948's own I2C port.  Only needed for pure-SPI builds. */

    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Try init with whatever s_i2c_addr / s_sda_gpio / s_scl_gpio / s_i2c_hz
   are currently set to. Tears down on failure so the caller can adjust
   and retry. */
static esp_err_t try_init(void)
{
    esp_err_t err = ensure_i2c();
    if (err != ESP_OK) return err;

    err = icm20948_init_chip();
    if (err != ESP_OK) {
        teardown_i2c(); /* clean slate for next attempt */
    }
    return err;
}

/* ------------------------------------------------------------------ */
esp_err_t icm20948_init(void)
{
    s_imu_ok   = false;
    s_i2c_addr = (uint8_t)ICM20948_I2C_ADDR; /* 0x68 */
    s_sda_gpio = ICM20948_SDA_GPIO;
    s_scl_gpio = ICM20948_SCL_GPIO;
    s_i2c_hz   = ICM20948_I2C_HZ;            /* 400k */

    vTaskDelay(pdMS_TO_TICKS(200));

    /* --- attempt 1: 0x68 @ 400k --- */
    esp_err_t err = try_init();
    if (err == ESP_OK) goto done;

    /* --- attempt 2: 0x69 @ 400k --- */
    ESP_LOGW(TAG, "retry at 0x69 ...");
    s_i2c_addr = 0x69;
    err = try_init();
    if (err == ESP_OK) goto done;

    /* --- attempt 3: 0x68 @ 100k --- */
    ESP_LOGW(TAG, "retry at 0x68, 100 kHz ...");
    s_i2c_addr = 0x68;
    s_i2c_hz   = 100000;
    err = try_init();
    if (err == ESP_OK) goto done;

    /* --- attempt 4: 0x69 @ 100k --- */
    ESP_LOGW(TAG, "retry at 0x69, 100 kHz ...");
    s_i2c_addr = 0x69;
    err = try_init();
    if (err == ESP_OK) goto done;

    /* --- attempts 5-8: swap SDA/SCL, repeat both addresses / speeds --- */
    ESP_LOGW(TAG, "retry with SDA/SCL swapped ...");
    s_sda_gpio = ICM20948_SCL_GPIO;
    s_scl_gpio = ICM20948_SDA_GPIO;

    s_i2c_addr = 0x68; s_i2c_hz = ICM20948_I2C_HZ;
    err = try_init();
    if (err == ESP_OK) goto done;

    s_i2c_addr = 0x69;
    err = try_init();
    if (err == ESP_OK) goto done;

    s_i2c_addr = 0x68; s_i2c_hz = 100000;
    err = try_init();
    if (err == ESP_OK) goto done;

    s_i2c_addr = 0x69;
    err = try_init();
    if (err == ESP_OK) goto done;

    /* All attempts failed — run a bus scan on the last successful bus
       setup so we can at least see what's on the wire. */
    ESP_LOGE(TAG, "all init attempts failed");

    /* Re-establish the bus just long enough to scan */
    s_sda_gpio = ICM20948_SDA_GPIO;
    s_scl_gpio = ICM20948_SCL_GPIO;
    s_i2c_addr = ICM20948_I2C_ADDR;
    s_i2c_hz   = 100000;
    if (ensure_i2c() == ESP_OK) {
        icm20948_i2c_scan_log();
        teardown_i2c();
    }
    return err;

done:
    imu_fusion_init();
    step_detect_init();
    cal_load_nvs();
    s_imu_ok = true;
    ESP_LOGI(TAG, "ICM20948 ready — SDA=%d SCL=%d addr=0x%02X @ %lu Hz",
             s_sda_gpio, s_scl_gpio, s_i2c_addr, (unsigned long)s_i2c_hz);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

esp_err_t icm20948_read_motion(int16_t *ax, int16_t *ay, int16_t *az,
                               int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (!s_imu_ok || !ax || !ay || !az || !gx || !gy || !gz) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = set_user_bank(0);
    if (err != ESP_OK) return err;

    uint8_t raw[12];
    err = read_regs(REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) return err;

    *ax = be16(&raw[0]);
    *ay = be16(&raw[2]);
    *az = be16(&raw[4]);
    *gx = be16(&raw[6]);
    *gy = be16(&raw[8]);
    *gz = be16(&raw[10]);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
static void icm20948_motion_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(ICM20948_FUSION_INTERVAL_MS);
    const uint32_t log_every = (ICM20948_SERIAL_LOG_MS + ICM20948_FUSION_INTERVAL_MS - 1)
                               / ICM20948_FUSION_INTERVAL_MS;
    int64_t last_us = esp_timer_get_time();
    uint32_t sample = 0;
    uint32_t read_fail = 0;

    if (!s_gyro_calibrated) {
        cal_reset_accumulators();
    }

    ESP_LOGI(TAG, "IMU fusion ~%.0f Hz, serial log every %d ms",
             1000.0f / (float)ICM20948_FUSION_INTERVAL_MS, ICM20948_SERIAL_LOG_MS);
    if (!s_gyro_calibrated) {
        ESP_LOGI(TAG, "keep helmet still for calibration (%d samples)...", GYRO_BIAS_SAMPLES);
    }

    for (;;) {
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us  = now_us;
        if (dt <= 0.0f || dt > 0.25f) {
            dt = (float)ICM20948_FUSION_INTERVAL_MS * 0.001f;
        }

        int16_t ax, ay, az, gx, gy, gz;
        if (icm20948_read_motion(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            float axg = (float)ax / ACCEL_LSB_PER_G;
            float ayg = (float)ay / ACCEL_LSB_PER_G;
            float azg = (float)az / ACCEL_LSB_PER_G;
            float gxr = (float)gx / GYRO_LSB_PER_DPS;
            float gyr = (float)gy / GYRO_LSB_PER_DPS;
            float gzr = (float)gz / GYRO_LSB_PER_DPS;

            if (!s_gyro_calibrated) {
                if (bias_stationary(vec3_mag(gxr, gyr, gzr), axg, ayg, azg)) {
                    s_gyro_cal_sum[0] += gxr;
                    s_gyro_cal_sum[1] += gyr;
                    s_gyro_cal_sum[2] += gzr;
                    s_accel_cal_sum[0] += axg;
                    s_accel_cal_sum[1] += ayg;
                    s_accel_cal_sum[2] += azg;
                    s_gyro_cal_count++;
                    if (s_gyro_cal_count >= (uint32_t)GYRO_BIAS_SAMPLES) {
                        cal_finish_from_accumulators();
                        portENTER_CRITICAL(&s_cal_mux);
                        s_cal_requested = false;
                        portEXIT_CRITICAL(&s_cal_mux);
                    }
                } else if (s_gyro_cal_count > 0) {
                    cal_reset_accumulators();
                }
            } else {
                float gxd = gxr - s_gyro_bias_dps[0];
                float gyd = gyr - s_gyro_bias_dps[1];
                float gzd = gzr - s_gyro_bias_dps[2];
                float gyro_mag = vec3_mag(gxd, gyd, gzd);
                s_yaw_rate_dps = gzd;

                apply_accel_cal(&axg, &ayg, &azg);

                if (bias_stationary(gyro_mag, axg, ayg, azg)) {
                    const float alpha = GYRO_BIAS_ONLINE_ALPHA;
                    s_gyro_bias_dps[0] = (1.0f - alpha) * s_gyro_bias_dps[0] + alpha * gxr;
                    s_gyro_bias_dps[1] = (1.0f - alpha) * s_gyro_bias_dps[1] + alpha * gyr;
                    s_gyro_bias_dps[2] = (1.0f - alpha) * s_gyro_bias_dps[2] + alpha * gzr;
                    gxd = gxr - s_gyro_bias_dps[0];
                    gyd = gyr - s_gyro_bias_dps[1];
                    gzd = gzr - s_gyro_bias_dps[2];
                    gyro_mag = vec3_mag(gxd, gyd, gzd);
                }

                step_detect_update(axg, ayg, azg, gyro_mag);

                if (s_imu_still) {
                    if (gyro_mag >= GYRO_STILL_OFF_DPS) {
                        s_imu_still = false;
                    }
                } else if (gyro_mag < GYRO_STILL_ON_DPS) {
                    s_imu_still = true;
                }

                float beta = fusion_accel_beta(axg, ayg, azg, gyro_mag);
                imu_fusion_update(gxd * DEG_TO_RAD, gyd * DEG_TO_RAD, gzd * DEG_TO_RAD,
                                  axg, ayg, azg, dt, beta);
            }

            sample++;
            if (sample == 1 || (sample % log_every) == 0) {
                float w, x, y, z;
                imu_fusion_get_display_quat(&w, &x, &y, &z);
                ESP_LOGI(TAG,
                         "ax=%d ay=%d az=%d | accel(g)=%.3f,%.3f,%.3f | "
                         "quat w=%.3f x=%.3f y=%.3f z=%.3f | cal=%d walk=%d",
                         ax, ay, az, axg, ayg, azg, w, x, y, z,
                         s_gyro_calibrated ? 1 : 0,
                         step_detect_is_walking() ? 1 : 0);
            }
        } else {
            read_fail++;
            if (read_fail == 1 || (read_fail % log_every) == 0) {
                ESP_LOGW(TAG, "read_motion failed (%lu times)", (unsigned long)read_fail);
            }
        }
        vTaskDelay(period);
    }
}

float icm20948_get_yaw_rate_dps(void)
{
    if (!s_gyro_calibrated) {
        return 0.0f;
    }
    return s_yaw_rate_dps;
}

bool icm20948_is_still(void)
{
    return s_gyro_calibrated && s_imu_still;
}

bool icm20948_is_calibrated(void)
{
    return s_gyro_calibrated;
}

i2c_master_bus_handle_t icm20948_get_i2c_bus(void)
{
    return s_i2c_installed ? s_bus : NULL;
}

esp_err_t icm20948_start_task(void)
{
    if (!s_imu_ok) return ESP_ERR_INVALID_STATE;
    BaseType_t ok = xTaskCreate(icm20948_motion_task, "icm20948", 5120, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}