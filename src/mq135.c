#include "mq135.h"

#include "config.h"
#include "sensor_status.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if MQ135_ENABLE

static const char *TAG = "mq135";

static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_channel;
static bool s_ready;

static int read_averaged_raw(void)
{
    int sum = 0;
    int count = 0;
    for (int i = 0; i < MQ135_ADC_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, s_channel, &raw) == ESP_OK) {
            sum += raw;
            count++;
        }
    }
    return count > 0 ? sum / count : 0;
}

static void mq135_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "polling AO on GPIO%d every %d ms", MQ135_ADC_GPIO, MQ135_POLL_MS);

    for (;;) {
        if (s_ready) {
            int raw = read_averaged_raw();
            sensor_status_set_mq135(raw, true);
        }
        vTaskDelay(pdMS_TO_TICKS(MQ135_POLL_MS));
    }
}

esp_err_t mq135_init(void)
{
    adc_unit_t unit = ADC_UNIT_1;
    esp_err_t err = adc_oneshot_io_to_channel(MQ135_ADC_GPIO, &unit, &s_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d is not an ADC pin: %s", MQ135_ADC_GPIO, esp_err_to_name(err));
        return err;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = unit,
    };
    err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    err = adc_oneshot_config_channel(s_adc, s_channel, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        return err;
    }

    s_ready = true;
    ESP_LOGI(TAG, "ready (GPIO%d, unit %d ch %d)", MQ135_ADC_GPIO, (int)unit, (int)s_channel);
    return ESP_OK;
}

void mq135_start_task(void)
{
    if (!s_ready) {
        return;
    }
    xTaskCreate(mq135_task, "mq135", 3072, NULL, 4, NULL);
}

#else

esp_err_t mq135_init(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

void mq135_start_task(void) {}

#endif
