#include "mq135.h"

#include "config.h"
#include "sensor_status.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if MQ_GAS_ENABLE

static const char *TAG = "mq_gas";

typedef struct {
    const char *name;
    int gpio;
    int status_index;
    adc_channel_t channel;
    bool configured;
} mq_sensor_t;

static mq_sensor_t s_sensors[MQ_GAS_SENSOR_COUNT] = {
    {"MQ-135", MQ135_ADC_GPIO, 0, ADC_CHANNEL_0, false},
    {"MQ-136", MQ136_ADC_GPIO, 1, ADC_CHANNEL_0, false},
    {"MQ-4", MQ4_ADC_GPIO, 2, ADC_CHANNEL_0, false},
    {"MQ-7", MQ7_ADC_GPIO, 3, ADC_CHANNEL_0, false},
};

static adc_oneshot_unit_handle_t s_adc;
static bool s_ready;

static int read_averaged_raw(adc_channel_t channel)
{
    int sum = 0;
    int count = 0;
    for (int i = 0; i < MQ_GAS_ADC_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, channel, &raw) == ESP_OK) {
            sum += raw;
            count++;
        }
    }
    return count > 0 ? sum / count : 0;
}

static void mq_gas_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "polling %d MQ sensors every %d ms", MQ_GAS_SENSOR_COUNT, MQ_GAS_POLL_MS);

    for (;;) {
        if (s_ready) {
            for (int i = 0; i < MQ_GAS_SENSOR_COUNT; i++) {
                if (!s_sensors[i].configured) {
                    continue;
                }
                int raw = read_averaged_raw(s_sensors[i].channel);
                sensor_status_set_mq_gas(s_sensors[i].status_index, raw, true);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MQ_GAS_POLL_MS));
    }
}

esp_err_t mq135_init(void)
{
    adc_unit_t unit = ADC_UNIT_1;
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = unit,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };

    int configured = 0;
    for (int i = 0; i < MQ_GAS_SENSOR_COUNT; i++) {
        mq_sensor_t *s = &s_sensors[i];
        adc_unit_t ch_unit = unit;
        adc_channel_t ch = s->channel;

        err = adc_oneshot_io_to_channel(s->gpio, &ch_unit, &ch);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "%s GPIO%d not ADC: %s", s->name, s->gpio, esp_err_to_name(err));
            s->configured = false;
            continue;
        }
        if (ch_unit != unit) {
            ESP_LOGW(TAG, "%s GPIO%d not on ADC1", s->name, s->gpio);
            s->configured = false;
            continue;
        }

        s->channel = ch;
        err = adc_oneshot_config_channel(s_adc, ch, &chan_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "%s ch config failed: %s", s->name, esp_err_to_name(err));
            s->configured = false;
            continue;
        }

        s->configured = true;
        configured++;
        ESP_LOGI(TAG, "%s AO on GPIO%d (ADC1 ch %d)", s->name, s->gpio, (int)ch);
    }

    if (configured == 0) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        return ESP_FAIL;
    }

    s_ready = true;
    ESP_LOGI(TAG, "%d/%d MQ gas sensors ready", configured, MQ_GAS_SENSOR_COUNT);
    return ESP_OK;
}

void mq135_start_task(void)
{
    if (!s_ready) {
        return;
    }
    xTaskCreate(mq_gas_task, "mq_gas", 3072, NULL, 4, NULL);
}

#else

esp_err_t mq135_init(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

void mq135_start_task(void) {}

#endif
