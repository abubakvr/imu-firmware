#include "oled_display.h"

#include "app_status.h"
#include "config.h"
#include "sensor_status.h"
#include "step_detect.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if OLED_ENABLE

static const char *TAG = "oled";

#define OLED_PAGES (OLED_HEIGHT / 8)
#define OLED_BUF_SIZE (OLED_WIDTH * OLED_PAGES)

#ifndef OLED_INVERT
#define OLED_INVERT 0
#endif

static spi_device_handle_t s_spi;
static uint8_t s_fb[OLED_BUF_SIZE];
static bool s_ready;

typedef struct {
    char c;
    uint8_t col[6];
} glyph6x8_t;

static const glyph6x8_t s_glyphs[] = {
    {' ', {0, 0, 0, 0, 0, 0}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E, 0}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00, 0}},
    {'2', {0x62, 0x51, 0x49, 0x49, 0x46, 0}},
    {'3', {0x22, 0x49, 0x49, 0x49, 0x36, 0}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10, 0}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39, 0}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30, 0}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03, 0}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36, 0}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E, 0}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00, 0}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00, 0}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08, 0}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E, 0}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36, 0}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22, 0}},
    {'D', {0x7F, 0x41, 0x41, 0x41, 0x3E, 0}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41, 0}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01, 0}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x3A, 0}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F, 0}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00, 0}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41, 0}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40, 0}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F, 0}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F, 0}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E, 0}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06, 0}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46, 0}},
    {'S', {0x26, 0x49, 0x49, 0x49, 0x32, 0}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01, 0}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F, 0}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F, 0}},
    {'W', {0x7F, 0x20, 0x18, 0x20, 0x7F, 0}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07, 0}},
    {'%', {0x23, 0x13, 0x08, 0x64, 0x62, 0}},
};

static esp_err_t spi_send(const uint8_t *data, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }
    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = data;
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t oled_cmd(uint8_t cmd)
{
    gpio_set_level((gpio_num_t)OLED_PIN_DC, 0);
    return spi_send(&cmd, 1);
}

static esp_err_t oled_cmds(const uint8_t *cmds, size_t n)
{
    esp_err_t err = ESP_OK;
    for (size_t i = 0; i < n && err == ESP_OK; i++) {
        err = oled_cmd(cmds[i]);
    }
    return err;
}

static esp_err_t oled_data(const uint8_t *data, size_t len)
{
    gpio_set_level((gpio_num_t)OLED_PIN_DC, 1);
    return spi_send(data, len);
}

static void oled_set_column_addr(uint8_t col)
{
#if OLED_CONTROLLER == OLED_CTRL_SH1106
    col = (uint8_t)(col + 2);
#endif
    oled_cmd((uint8_t)(0x00 | (col & 0x0F)));
    oled_cmd((uint8_t)(0x10 | ((col >> 4) & 0x0F)));
}

static esp_err_t panel_flush(void)
{
    esp_err_t err = ESP_OK;
    for (int page = 0; page < OLED_PAGES && err == ESP_OK; page++) {
        err = oled_cmd((uint8_t)(0xB0 | page));
        if (err != ESP_OK) {
            break;
        }
        oled_set_column_addr(0);
        err = oled_data(&s_fb[(size_t)page * OLED_WIDTH], OLED_WIDTH);
    }
    return err;
}

static bool glyph_lookup(char ch, uint8_t out[6])
{
    if (ch >= 'a' && ch <= 'z') {
        ch = (char)(ch - 'a' + 'A');
    }
    for (size_t i = 0; i < sizeof(s_glyphs) / sizeof(s_glyphs[0]); i++) {
        if (s_glyphs[i].c == ch) {
            memcpy(out, s_glyphs[i].col, 6);
            return true;
        }
    }
    memset(out, 0, 6);
    return false;
}

static void fb_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

static void fb_fill(bool on)
{
    memset(s_fb, on ? 0xFF : 0x00, sizeof(s_fb));
}

static void fb_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return;
    }
    size_t idx = (size_t)x + (size_t)(y / 8) * OLED_WIDTH;
    uint8_t mask = (uint8_t)(1U << (y % 8));
    if (on) {
        s_fb[idx] |= mask;
    } else {
        s_fb[idx] &= (uint8_t)~mask;
    }
}

static void fb_draw_char(int x, int y, char ch)
{
    uint8_t cols[6];
    glyph_lookup(ch, cols);
    for (int col = 0; col < 6; col++) {
        uint8_t bits = cols[col];
        for (int row = 0; row < 8; row++) {
            if (bits & (1U << row)) {
                fb_set_pixel(x + col, y + row, true);
            }
        }
    }
}

static void fb_draw_text(int x, int y, const char *text)
{
    int cx = x;
    for (const char *p = text; *p; p++) {
        fb_draw_char(cx, y, *p);
        cx += 7;
    }
}

static esp_err_t oled_hw_reset(void)
{
    if (OLED_PIN_RST < 0) {
        return ESP_OK;
    }
    gpio_config_t rst = {
        .pin_bit_mask = 1ULL << OLED_PIN_RST,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&rst);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level((gpio_num_t)OLED_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

static esp_err_t oled_controller_init(void)
{
    const uint8_t mux = (uint8_t)(OLED_HEIGHT - 1);
    const uint8_t com_pins = (OLED_HEIGHT == 64) ? 0x12 : 0x02;

    static const uint8_t common[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, mux,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x02,
        0xA1,
        0xC8,
        0xDA, com_pins,
        0x81, 0xCF,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
    };

    esp_err_t err = oled_cmds(common, sizeof(common));
    if (err != ESP_OK) {
        return err;
    }
    err = oled_cmd(OLED_INVERT ? 0xA7 : 0xA6);
    if (err != ESP_OK) {
        return err;
    }
    return oled_cmd(0xAF);
}

static esp_err_t oled_self_test(void)
{
    fb_fill(true);
    esp_err_t err = panel_flush();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "self-test flush: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(250));
    fb_clear();
    err = panel_flush();
    return err;
}

static void render_status_screen(void)
{
    char line[22];
    bool wifi = app_status_wifi_connected();
    bool imu = app_status_imu_ok();
    bool walking = step_detect_is_walking();
    uint32_t steps = step_detect_get_step_count();

    fb_clear();

    if (sensor_status_temp_valid()) {
        snprintf(line, sizeof(line), "T %.1fC", sensor_status_temp_c());
    } else {
        snprintf(line, sizeof(line), "T --");
    }
    fb_draw_text(0, 0, line);

    if (sensor_status_vitals_valid() && sensor_status_hr_bpm() > 0) {
        snprintf(line, sizeof(line), "HR %d", sensor_status_hr_bpm());
    } else {
        snprintf(line, sizeof(line), "HR --");
    }
    fb_draw_text(0, 10, line);

    if (sensor_status_vitals_valid() && sensor_status_spo2_pct() > 0) {
        snprintf(line, sizeof(line), "SpO2 %d%%", sensor_status_spo2_pct());
    } else {
        snprintf(line, sizeof(line), "SpO2 --");
    }
    fb_draw_text(0, 20, line);

#if MQ_GAS_ENABLE
    if (sensor_status_mq_gas_valid(0)) {
        snprintf(line, sizeof(line), "MQ135 %d", sensor_status_mq_gas_raw(0));
    } else {
        snprintf(line, sizeof(line), "MQ135 --");
    }
    fb_draw_text(0, 30, line);
#endif

    fb_draw_text(0, 40, walking ? "WALKING" : "STILL");

    snprintf(line, sizeof(line), "Steps %lu", (unsigned long)steps);
    fb_draw_text(0, 48, line);

    snprintf(line, sizeof(line), "WiFi %s IMU %s", wifi ? "OK" : "--", imu ? "OK" : "--");
    fb_draw_text(0, 56, line);
    panel_flush();
}

static void oled_task(void *arg)
{
    (void)arg;
    while (true) {
        if (s_ready) {
            render_status_screen();
        }
        vTaskDelay(pdMS_TO_TICKS(OLED_REFRESH_MS));
    }
}

esp_err_t oled_display_init(void)
{
    const spi_host_device_t host = (spi_host_device_t)OLED_SPI_HOST_ID;

    gpio_config_t dc = {
        .pin_bit_mask = 1ULL << OLED_PIN_DC,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&dc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DC gpio: %s", esp_err_to_name(err));
        return err;
    }

    err = oled_hw_reset();
    if (err != ESP_OK) {
        return err;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = OLED_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = OLED_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = OLED_BUF_SIZE + 16,
    };
    err = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = OLED_SPI_HZ,
        .mode = 0,
        .spics_io_num = OLED_PIN_CS,
        .queue_size = 4,
    };
    err = spi_bus_add_device(host, &devcfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    err = oled_controller_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "controller init: %s", esp_err_to_name(err));
        return err;
    }

    err = oled_self_test();
    if (err != ESP_OK) {
        return err;
    }

    s_ready = true;
    ESP_LOGI(TAG, "OLED ready %s SPI %dx%d MOSI=%d CLK=%d DC=%d CS=%d RST=%d",
#if OLED_CONTROLLER == OLED_CTRL_SH1106
             "SH1106",
#else
             "SSD1306",
#endif
             OLED_WIDTH, OLED_HEIGHT, OLED_PIN_MOSI, OLED_PIN_CLK, OLED_PIN_DC, OLED_PIN_CS,
             OLED_PIN_RST);
    return ESP_OK;
}

void oled_display_show_boot(const char *line1, const char *line2)
{
    if (!s_ready) {
        return;
    }
    fb_clear();
    if (line1) {
        fb_draw_text(0, 20, line1);
    }
    if (line2) {
        fb_draw_text(0, 34, line2);
    }
    panel_flush();
}

esp_err_t oled_display_start_task(void)
{
    BaseType_t ok = xTaskCreate(oled_task, "oled", 3072, NULL, 3, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

#else

esp_err_t oled_display_init(void)
{
    return ESP_OK;
}

esp_err_t oled_display_start_task(void)
{
    return ESP_OK;
}

void oled_display_show_boot(const char *line1, const char *line2)
{
    (void)line1;
    (void)line2;
}

#endif
