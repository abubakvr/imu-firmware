#include "health_sensors.h"

#include "config.h"
#include "sensor_status.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if HEALTH_SENSORS_ENABLE

static const char *TAG = "health";

#define I2C_TIMEOUT_MS 100

/* MAX30102 */
#define MAX30102_ADDR        ((uint8_t)MAX30102_I2C_ADDR)
#define MAX30102_REG_MODE    0x09U
#define MAX30102_REG_SPO2    0x0AU
#define MAX30102_REG_LED1    0x0CU
#define MAX30102_REG_LED2    0x0DU
#define MAX30102_REG_FIFO_CFG  0x08U
#define MAX30102_REG_FIFO_WR   0x04U
#define MAX30102_REG_FIFO_RD   0x06U
#define MAX30102_REG_FIFO_DATA 0x07U
#define MAX30102_REG_PART_ID   0xFFU
#define MAX30102_PART_ID       0x15U

#define MAX30205_REG_TEMP   0x00U
#define MAX30205_REG_CONFIG 0x01U

/* 7-bit addresses from MAX30205 Table 1 (A0/A1/A2 → 0x90…0xA8). */
static const uint8_t s_max30205_addrs[32] = {
    0x4F, 0x4E, 0x4D, 0x4C, 0x4B, 0x4A, 0x49, 0x48, 0x41, 0x40, 0x43, 0x42,
    0x5A, 0x5B, 0x53, 0x52, 0x58, 0x59, 0x51, 0x50, 0x45, 0x44, 0x47, 0x46,
    0x5E, 0x5F, 0x57, 0x56, 0x5C, 0x5D, 0x55, 0x54,
};

typedef enum {
    TEMP_FMT_NONE = 0,
    TEMP_FMT_MAX30205,
    TEMP_FMT_LM75,
} temp_fmt_t;

/* Slower clock for MAX30205 / CJMCU-30205 on a shared bus with IMU + MAX30102. */
#define TEMP_I2C_HZ 50000
#define TEMP_CONV_DELAY_MS 100

#define FIFO_BUF_SAMPLES   100
#define VITALS_WINDOW_MS   4000
#define HEALTH_POLL_MS     20
/* 100 SPS with 4-sample FIFO average (see FIFO_CFG 0x4F) */
#define MAX30102_SAMPLE_HZ 25.0f
#define FINGER_IR_MIN      50000.0f

static i2c_master_dev_handle_t s_max_dev;
static i2c_master_dev_handle_t s_temp_dev;
static i2c_master_bus_handle_t s_imu_bus;
static temp_fmt_t s_temp_fmt;
static bool s_max_ok;
static bool s_temp_ok;
static uint8_t s_temp_addr;

static uint32_t s_ir_buf[FIFO_BUF_SAMPLES];
static uint32_t s_red_buf[FIFO_BUF_SAMPLES];
static int s_buf_len;
static int64_t s_buf_start_us;

static esp_err_t max_write_u8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_max_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t max_read_u8(uint8_t reg, uint8_t *out)
{
    return i2c_master_transmit_receive(s_max_dev, &reg, 1, out, 1, I2C_TIMEOUT_MS);
}

static esp_err_t max_read_fifo(uint32_t *red, uint32_t *ir)
{
    uint8_t reg = MAX30102_REG_FIFO_DATA;
    uint8_t raw[6];
    esp_err_t err = i2c_master_transmit_receive(s_max_dev, &reg, 1, raw, sizeof(raw), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    *red = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | (uint32_t)raw[2];
    *ir  = ((uint32_t)raw[3] << 16) | ((uint32_t)raw[4] << 8) | (uint32_t)raw[5];
    *red &= 0x03FFFFU;
    *ir  &= 0x03FFFFU;
    return ESP_OK;
}

static esp_err_t max30102_init_chip(void)
{
    uint8_t part = 0;
    esp_err_t err = max_read_u8(MAX30102_REG_PART_ID, &part);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MAX30102 probe failed: %s", esp_err_to_name(err));
        return err;
    }
    if (part != MAX30102_PART_ID) {
        ESP_LOGW(TAG, "MAX30102 PART_ID=0x%02X (expected 0x%02X)", part, MAX30102_PART_ID);
        return ESP_ERR_NOT_FOUND;
    }

    err = max_write_u8(MAX30102_REG_MODE, 0x40U); /* reset */
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* SpO2 mode, 100 Hz sample rate, 411 us pulse width, 18-bit ADC */
    err = max_write_u8(MAX30102_REG_MODE, 0x03U);
    if (err != ESP_OK) {
        return err;
    }
    err = max_write_u8(MAX30102_REG_SPO2, 0x27U);
    if (err != ESP_OK) {
        return err;
    }
    err = max_write_u8(MAX30102_REG_LED1, 0x24U);
    if (err != ESP_OK) {
        return err;
    }
    err = max_write_u8(MAX30102_REG_LED2, 0x24U);
    if (err != ESP_OK) {
        return err;
    }
    err = max_write_u8(MAX30102_REG_FIFO_CFG, 0x4FU); /* avg 4, rollover, almost full 17 */
    if (err != ESP_OK) {
        return err;
    }
    err = max_write_u8(MAX30102_REG_MODE, 0x03U); /* start */
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "MAX30102 ready at 0x%02X", MAX30102_ADDR);
    return ESP_OK;
}

static esp_err_t temp_read_reg16(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t raw[2])
{
    esp_err_t err = i2c_master_transmit(dev, &reg, 1, I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        err = i2c_master_receive(dev, raw, 2, I2C_TIMEOUT_MS);
    }
    if (err != ESP_OK) {
        err = i2c_master_transmit_receive(dev, &reg, 1, raw, 2, I2C_TIMEOUT_MS);
    }
    return err;
}

static float temp_raw_to_c_max30205(const uint8_t raw[2])
{
    int16_t data = (int16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
    return (float)data / 256.0f;
}

static float temp_raw_to_c_lm75(const uint8_t raw[2])
{
    int16_t data = (int16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
    return ((float)(data >> 7)) * 0.5f;
}

static bool temp_c_plausible(float c)
{
    return c >= -40.0f && c <= 125.0f;
}

static bool temp_skip_addr(uint8_t addr)
{
    if (addr == (uint8_t)ICM20948_I2C_ADDR || addr == 0x69U) {
        return true;
    }
    if (addr == (uint8_t)MAX30102_I2C_ADDR) {
        return true;
    }
    return false;
}

static esp_err_t temp_decode_raw(const uint8_t raw[2], temp_fmt_t *fmt_out, float *out_c)
{
    float tc_max = temp_raw_to_c_max30205(raw);
    float tc_lm = temp_raw_to_c_lm75(raw);

    if (temp_c_plausible(tc_max)) {
        *fmt_out = TEMP_FMT_MAX30205;
        *out_c = tc_max;
        return ESP_OK;
    }
    if (temp_c_plausible(tc_lm)) {
        *fmt_out = TEMP_FMT_LM75;
        *out_c = tc_lm;
        return ESP_OK;
    }
    return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t max30205_read_c_dev(i2c_master_dev_handle_t dev, temp_fmt_t fmt, float *out_c)
{
    uint8_t raw[2];
    esp_err_t err = temp_read_reg16(dev, MAX30205_REG_TEMP, raw);
    if (err != ESP_OK) {
        return err;
    }

    temp_fmt_t decoded = TEMP_FMT_NONE;
    err = temp_decode_raw(raw, &decoded, out_c);
    if (err != ESP_OK) {
        return err;
    }
    (void)fmt;
    return ESP_OK;
}

static esp_err_t max30205_init_dev(i2c_master_dev_handle_t dev)
{
    uint8_t buf[2] = {MAX30205_REG_CONFIG, 0x00U};
    esp_err_t err = i2c_master_transmit(dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(TEMP_CONV_DELAY_MS));
    return ESP_OK;
}

static esp_err_t temp_read_c(float *out_c)
{
    if (!s_temp_dev || s_temp_fmt == TEMP_FMT_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    return max30205_read_c_dev(s_temp_dev, s_temp_fmt, out_c);
}

static void log_max30205_sample(i2c_master_bus_handle_t bus, uint8_t addr, const char *ctx)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = TEMP_I2C_HZ,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK) {
        return;
    }

    uint8_t raw[2] = {0};
    float tc_max = NAN;
    float tc_lm = NAN;
    esp_err_t err = temp_read_reg16(dev, MAX30205_REG_TEMP, raw);
    if (err == ESP_OK) {
        tc_max = temp_raw_to_c_max30205(raw);
        tc_lm = temp_raw_to_c_lm75(raw);
    }
    ESP_LOGI(TAG, "%s 0x%02X: err=%s raw=%02X %02X max30205=%.2f lm75=%.2f",
             ctx, addr, esp_err_to_name(err), raw[0], raw[1], tc_max, tc_lm);
    i2c_master_bus_rm_device(dev);
}

static void max30205_addr_scan_log(i2c_master_bus_handle_t bus)
{
    static const uint8_t quick_addrs[] = {
        (uint8_t)TEMP_SENSOR_I2C_ADDR,
        0x48U,
        0x4DU,
    };
    int n = 0;

    ESP_LOGI(TAG, "CJMCU-30205 address ACKs:");
    for (size_t i = 0; i < sizeof(quick_addrs) / sizeof(quick_addrs[0]); i++) {
        uint8_t addr = quick_addrs[i];
        if (temp_skip_addr(addr)) {
            continue;
        }
        if (i2c_master_probe(bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "  0x%02X", addr);
            n++;
        }
    }
    if (n == 0) {
        ESP_LOGW(TAG, "  none (CJMCU not responding — check VIN/GND on module)");
    }
}

static void i2c_quick_check_log(i2c_master_bus_handle_t bus)
{
    static const uint8_t addrs[] = {
        (uint8_t)ICM20948_I2C_ADDR,
        0x69U,
        (uint8_t)TEMP_SENSOR_I2C_ADDR,
        (uint8_t)MAX30102_I2C_ADDR,
    };

    ESP_LOGI(TAG, "I2C quick check (shared bus SDA=GPIO%d SCL=GPIO%d):",
             ICM20948_SDA_GPIO, ICM20948_SCL_GPIO);
    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
        uint8_t addr = addrs[i];
        if (i2c_master_probe(bus, addr, I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "  ACK 0x%02X", addr);
        } else {
            ESP_LOGI(TAG, "  --- 0x%02X", addr);
        }
    }
}

void health_sensors_i2c_diagnose(i2c_master_bus_handle_t bus)
{
    if (!bus) {
        ESP_LOGW(TAG, "I2C diagnose: bus not ready");
        return;
    }
    i2c_quick_check_log(bus);
    log_max30205_sample(bus, (uint8_t)TEMP_SENSOR_I2C_ADDR, "CJMCU-30205");
}

static bool temp_probe_device(i2c_master_dev_handle_t dev, temp_fmt_t *fmt_out, float *c_out)
{
    uint8_t raw[2];

    if (max30205_init_dev(dev) != ESP_OK) {
        return false;
    }
    if (temp_read_reg16(dev, MAX30205_REG_TEMP, raw) != ESP_OK) {
        return false;
    }
    if (temp_decode_raw(raw, fmt_out, c_out) != ESP_OK) {
        return false;
    }
    return true;
}

static bool temp_try_addr(i2c_master_bus_handle_t bus, uint8_t addr, float *tc_out)
{
    if (temp_skip_addr(addr)) {
        return false;
    }
    if (i2c_master_probe(bus, addr, I2C_TIMEOUT_MS) != ESP_OK) {
        if (addr == (uint8_t)TEMP_SENSOR_I2C_ADDR) {
            ESP_LOGW(TAG, "no I2C ACK at 0x%02X (CJMCU default address)", addr);
        }
        return false;
    }

    i2c_device_config_t tmp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = TEMP_I2C_HZ,
    };
    i2c_master_dev_handle_t probe = NULL;
    if (i2c_master_bus_add_device(bus, &tmp_cfg, &probe) != ESP_OK) {
        return false;
    }

    temp_fmt_t fmt = TEMP_FMT_NONE;
    float tc = NAN;
    bool ok = temp_probe_device(probe, &fmt, &tc);
    if (ok) {
        s_temp_dev = probe;
        s_temp_addr = addr;
        s_temp_fmt = fmt;
        s_temp_ok = true;
        *tc_out = tc;
        ESP_LOGI(TAG, "CJMCU-30205 at 0x%02X (%.2f C, fmt=%s)", addr, tc,
                 fmt == TEMP_FMT_LM75 ? "lm75" : "max30205");
    } else {
        if (addr != 0x4BU) {
            uint8_t raw[2] = {0};
            esp_err_t err = temp_read_reg16(probe, MAX30205_REG_TEMP, raw);
            ESP_LOGD(TAG, "0x%02X ACK but not MAX30205: %s raw=%02X %02X", addr,
                     esp_err_to_name(err), raw[0], raw[1]);
        }
        i2c_master_bus_rm_device(probe);
    }
    return ok;
}

static bool temp_search_on_bus(i2c_master_bus_handle_t bus)
{
    if (!bus) {
        return false;
    }

    float tc = NAN;
    if (temp_try_addr(bus, (uint8_t)TEMP_SENSOR_I2C_ADDR, &tc)) {
        sensor_status_set_temp(tc, true);
        return true;
    }

    for (size_t ai = 0; ai < sizeof(s_max30205_addrs) / sizeof(s_max30205_addrs[0]); ai++) {
        uint8_t addr = s_max30205_addrs[ai];
        if (addr == (uint8_t)TEMP_SENSOR_I2C_ADDR) {
            continue;
        }
        if (temp_try_addr(bus, addr, &tc)) {
            sensor_status_set_temp(tc, true);
            return true;
        }
    }

    return false;
}

static int max30102_fifo_count(void)
{
    uint8_t wr = 0;
    uint8_t rd = 0;
    if (max_read_u8(MAX30102_REG_FIFO_WR, &wr) != ESP_OK) {
        return 0;
    }
    if (max_read_u8(MAX30102_REG_FIFO_RD, &rd) != ESP_OK) {
        return 0;
    }
    return (int)((wr - rd) & 0x1FU);
}

static void append_fifo_sample(uint32_t red, uint32_t ir)
{
    if (s_buf_len < FIFO_BUF_SAMPLES) {
        s_red_buf[s_buf_len] = red;
        s_ir_buf[s_buf_len] = ir;
        s_buf_len++;
        return;
    }
    memmove(s_red_buf, &s_red_buf[1], (size_t)(FIFO_BUF_SAMPLES - 1) * sizeof(s_red_buf[0]));
    memmove(s_ir_buf, &s_ir_buf[1], (size_t)(FIFO_BUF_SAMPLES - 1) * sizeof(s_ir_buf[0]));
    s_red_buf[FIFO_BUF_SAMPLES - 1] = red;
    s_ir_buf[FIFO_BUF_SAMPLES - 1] = ir;
}

static float buf_mean(const uint32_t *buf, int n)
{
    if (n <= 0) {
        return 0.0f;
    }
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += (double)buf[i];
    }
    return (float)(sum / (double)n);
}

static int estimate_hr_autocorr(const float *ac, int n, float sample_hz)
{
    if (n < 40 || sample_hz <= 0.0f) {
        return 0;
    }

    int min_lag = (int)(sample_hz * 60.0f / 200.0f + 0.5f);
    if (min_lag < 3) {
        min_lag = 3;
    }
    int max_lag = (int)(sample_hz * 60.0f / 40.0f + 0.5f);
    if (max_lag >= n / 2) {
        max_lag = n / 2 - 1;
    }
    if (max_lag <= min_lag) {
        return 0;
    }

    float best_corr = 0.0f;
    int best_lag = 0;
    for (int lag = min_lag; lag <= max_lag; lag++) {
        float num = 0.0f;
        float e0 = 0.0f;
        float e1 = 0.0f;
        for (int i = 0; i < n - lag; i++) {
            num += ac[i] * ac[i + lag];
            e0 += ac[i] * ac[i];
            e1 += ac[i + lag] * ac[i + lag];
        }
        float den = sqrtf(e0 * e1);
        float corr = (den > 1e-6f) ? (num / den) : 0.0f;
        if (corr > best_corr) {
            best_corr = corr;
            best_lag = lag;
        }
    }

    if (best_lag <= 0 || best_corr < 0.20f) {
        return 0;
    }

    float bpm = 60.0f * sample_hz / (float)best_lag;
    if (bpm < 40.0f || bpm > 200.0f) {
        return 0;
    }
    return (int)(bpm + 0.5f);
}

static void compute_vitals(int *hr_bpm, int *spo2_pct, bool *valid)
{
    *hr_bpm = 0;
    *spo2_pct = 0;
    *valid = false;

    if (s_buf_len < 40) {
        return;
    }

    float ir_mean = buf_mean(s_ir_buf, s_buf_len);
    if (ir_mean < FINGER_IR_MIN) {
        return;
    }

    float ir_ac[FIFO_BUF_SAMPLES];
    float red_ac[FIFO_BUF_SAMPLES];
    float ir_dc = ir_mean;
    float red_dc = buf_mean(s_red_buf, s_buf_len);

    for (int i = 0; i < s_buf_len; i++) {
        ir_ac[i] = (float)s_ir_buf[i] - ir_dc;
        red_ac[i] = (float)s_red_buf[i] - red_dc;
    }

    *hr_bpm = estimate_hr_autocorr(ir_ac, s_buf_len, MAX30102_SAMPLE_HZ);

    float ir_rms = 0.0f;
    float red_rms = 0.0f;
    for (int i = 0; i < s_buf_len; i++) {
        ir_rms += ir_ac[i] * ir_ac[i];
        red_rms += red_ac[i] * red_ac[i];
    }
    ir_rms = sqrtf(ir_rms / (float)s_buf_len);
    red_rms = sqrtf(red_rms / (float)s_buf_len);

    if (ir_rms > 1.0f && red_rms > 1.0f && red_dc > 1.0f) {
        float r_ratio = (red_rms / red_dc) / (ir_rms / ir_dc);
        float spo2 = 110.0f - 25.0f * r_ratio;
        if (spo2 < 70.0f) {
            spo2 = 70.0f;
        }
        if (spo2 > 100.0f) {
            spo2 = 100.0f;
        }
        *spo2_pct = (int)(spo2 + 0.5f);
    }

    *valid = (*hr_bpm > 0) || (*spo2_pct > 0);
}

static void health_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(HEALTH_POLL_MS);
    int64_t last_temp_us = 0;

    s_buf_len = 0;
    s_buf_start_us = esp_timer_get_time();

    ESP_LOGI(TAG, "health task running (place finger on MAX30102 for HR/SpO2)");

    int64_t last_temp_retry_us = esp_timer_get_time();

    for (;;) {
        int64_t now = esp_timer_get_time();

        if (!s_temp_ok && s_imu_bus &&
            (now - last_temp_retry_us) >= (int64_t)HEALTH_TEMP_RETRY_MS * 1000) {
            if (temp_search_on_bus(s_imu_bus)) {
                ESP_LOGI(TAG, "CJMCU-30205 found on retry");
            }
            last_temp_retry_us = now;
        }

        if (s_temp_ok && (now - last_temp_us) >= 500000) {
            float tc = NAN;
            if (temp_read_c(&tc) == ESP_OK && temp_c_plausible(tc)) {
                sensor_status_set_temp(tc, true);
            }
            last_temp_us = now;
        }

        if (s_max_ok) {
            int pending = max30102_fifo_count();
            if (pending <= 0) {
                pending = 1;
            }
            if (pending > 8) {
                pending = 8;
            }

            for (int i = 0; i < pending; i++) {
                uint32_t red = 0;
                uint32_t ir = 0;
                if (max_read_fifo(&red, &ir) != ESP_OK) {
                    break;
                }
                append_fifo_sample(red, ir);
            }

            if ((now - s_buf_start_us) >= (int64_t)VITALS_WINDOW_MS * 1000) {
                int hr = 0;
                int spo2 = 0;
                bool valid = false;
                compute_vitals(&hr, &spo2, &valid);
                sensor_status_set_vitals(hr, spo2, valid);
                s_buf_len = 0;
                s_buf_start_us = now;
            }
        }

        vTaskDelay(period);
    }
}

esp_err_t health_sensors_init(i2c_master_bus_handle_t bus)
{
    s_max_ok = false;
    s_temp_ok = false;
    s_temp_fmt = TEMP_FMT_NONE;
    s_temp_dev = NULL;
    s_imu_bus = bus;
    s_temp_addr = 0;

    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Temperature first (before MAX30102 FIFO traffic). */
    max30205_addr_scan_log(bus);
    if (!temp_search_on_bus(bus)) {
        health_sensors_i2c_diagnose(bus);
        ESP_LOGW(TAG, "CJMCU-30205 not found on shared I2C");
        ESP_LOGW(TAG, "  SDA→GPIO%d SCL→GPIO%d VIN→3.3V (parallel with IMU/HR)",
                 ICM20948_SDA_GPIO, ICM20948_SCL_GPIO);
    }

    i2c_device_config_t max_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MAX30102_ADDR,
        .scl_speed_hz    = ICM20948_I2C_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &max_cfg, &s_max_dev);
    if (err == ESP_OK) {
        if (max30102_init_chip() == ESP_OK) {
            s_max_ok = true;
        } else if (s_max_dev) {
            i2c_master_bus_rm_device(s_max_dev);
            s_max_dev = NULL;
        }
    } else {
        ESP_LOGW(TAG, "MAX30102 add device: %s", esp_err_to_name(err));
    }

    if (!s_max_ok && !s_temp_ok) {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

esp_err_t health_sensors_start_task(void)
{
    if (!s_max_ok && !s_temp_ok) {
        return ESP_ERR_INVALID_STATE;
    }
    BaseType_t ok = xTaskCreate(health_task, "health", 4096, NULL, 4, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

bool health_sensors_temp_ok(void)
{
    return s_temp_ok;
}

bool health_sensors_max30102_ok(void)
{
    return s_max_ok;
}

#else

void health_sensors_i2c_diagnose(i2c_master_bus_handle_t bus)
{
    (void)bus;
}

esp_err_t health_sensors_init(i2c_master_bus_handle_t bus)
{
    (void)bus;
    sensor_status_init();
    return ESP_OK;
}

esp_err_t health_sensors_start_task(void)
{
    return ESP_OK;
}

bool health_sensors_temp_ok(void)
{
    return false;
}

bool health_sensors_max30102_ok(void)
{
    return false;
}

#endif
