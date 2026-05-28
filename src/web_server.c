#include "web_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20948.h"
#include "imu_fusion.h"
#include "sensor_status.h"
#include "step_detect.h"
#include "config.h"
#include "index_html.h"

static const char *TAG = "web";
static httpd_handle_t s_server;
static int s_ws_fd = -1;
static bool s_ws_send_pending;
static portMUX_TYPE s_ws_mux = portMUX_INITIALIZER_UNLOCKED;

#define WS_MSG_MAX 320
#define WS_BROADCAST_MS 20

typedef struct {
    httpd_handle_t hd;
    int fd;
    char msg[WS_MSG_MAX];
} ws_send_arg_t;

static int ws_fd_get(void)
{
    portENTER_CRITICAL(&s_ws_mux);
    int fd = s_ws_fd;
    portEXIT_CRITICAL(&s_ws_mux);
    return fd;
}

static void ws_fd_set(int fd)
{
    portENTER_CRITICAL(&s_ws_mux);
    s_ws_fd = fd;
    portEXIT_CRITICAL(&s_ws_mux);
}

static void ws_fd_clear_if(int fd)
{
    portENTER_CRITICAL(&s_ws_mux);
    if (s_ws_fd == fd) {
        s_ws_fd = -1;
    }
    portEXIT_CRITICAL(&s_ws_mux);
}

static bool ws_client_alive(httpd_handle_t hd, int fd)
{
    if (!hd || fd < 0) {
        return false;
    }
    return httpd_ws_get_fd_info(hd, fd) == HTTPD_WS_CLIENT_WEBSOCKET;
}

static void httpd_close_fn(httpd_handle_t hd, int sockfd)
{
    ws_fd_clear_if(sockfd);
}

static int build_ws_json(char *buf, size_t buf_len)
{
    float w, x, y, z;
    bool walking = step_detect_is_walking();
    uint32_t steps = step_detect_get_step_count();
    float gz_dps = icm20948_get_yaw_rate_dps();
    imu_fusion_get_display_quat(&w, &x, &y, &z);

    bool temp_ok = sensor_status_temp_valid();
    float temp_c = sensor_status_temp_c();
    int hr = sensor_status_hr_bpm();
    int spo2 = sensor_status_spo2_pct();

    bool still = icm20948_is_still();
    bool calibrated = icm20948_is_calibrated();
    int n = snprintf(buf, buf_len,
                     "{\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"gz\":%.2f,"
                     "\"walking\":%s,\"still\":%s,\"calibrated\":%s,\"steps\":%lu",
                     w, x, y, z, gz_dps, walking ? "true" : "false", still ? "true" : "false",
                     calibrated ? "true" : "false", (unsigned long)steps);
    if (n < 0) {
        n = 0;
    }
    if (temp_ok && (size_t)n < buf_len - 16U) {
        n += snprintf(buf + n, buf_len - (size_t)n, ",\"temp\":%.1f", temp_c);
    }
    if (hr > 0 && (size_t)n < buf_len - 12U) {
        n += snprintf(buf + n, buf_len - (size_t)n, ",\"hr\":%d", hr);
    }
    if (spo2 > 0 && (size_t)n < buf_len - 12U) {
        n += snprintf(buf + n, buf_len - (size_t)n, ",\"spo2\":%d", spo2);
    }
#if MQ_GAS_ENABLE
    if (sensor_status_mq_gas_valid(0) && (size_t)n < buf_len - 14U) {
        n += snprintf(buf + n, buf_len - (size_t)n, ",\"mq135\":%d",
                      sensor_status_mq_gas_raw(0));
    }
    if (sensor_status_mq_gas_valid(1) && (size_t)n < buf_len - 14U) {
        n += snprintf(buf + n, buf_len - (size_t)n, ",\"mq136\":%d",
                      sensor_status_mq_gas_raw(1));
    }
    if (sensor_status_mq_gas_valid(2) && (size_t)n < buf_len - 12U) {
        n += snprintf(buf + n, buf_len - (size_t)n, ",\"mq4\":%d",
                      sensor_status_mq_gas_raw(2));
    }
    if (sensor_status_mq_gas_valid(3) && (size_t)n < buf_len - 12U) {
        n += snprintf(buf + n, buf_len - (size_t)n, ",\"mq7\":%d",
                      sensor_status_mq_gas_raw(3));
    }
#endif
    if ((size_t)n >= buf_len - 1U) {
        n = (int)buf_len - 2;
    }
    buf[n] = '}';
    buf[n + 1] = '\0';
    return n + 1;
}

static void ws_send_work(void *arg)
{
    ws_send_arg_t *a = (ws_send_arg_t *)arg;
    s_ws_send_pending = false;

    if (!ws_client_alive(a->hd, a->fd)) {
        ws_fd_clear_if(a->fd);
        free(a);
        return;
    }

    httpd_ws_frame_t frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)a->msg,
        .len = strlen(a->msg),
    };
    esp_err_t err = httpd_ws_send_frame_async(a->hd, a->fd, &frame);
    if (err != ESP_OK) {
        ws_fd_clear_if(a->fd);
        ESP_LOGD(TAG, "WebSocket send dropped (fd=%d): %s", a->fd, esp_err_to_name(err));
    }
    free(a);
}

static esp_err_t queue_quat_send(httpd_handle_t hd, int fd)
{
    if (!hd || fd < 0 || !ws_client_alive(hd, fd)) {
        if (fd >= 0) {
            ws_fd_clear_if(fd);
        }
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ws_send_pending) {
        return ESP_OK;
    }

    ws_send_arg_t *a = calloc(1, sizeof(*a));
    if (!a) {
        return ESP_ERR_NO_MEM;
    }

    build_ws_json(a->msg, sizeof(a->msg));
    a->hd = hd;
    a->fd = fd;

    s_ws_send_pending = true;
    esp_err_t err = httpd_queue_work(hd, ws_send_work, a);
    if (err != ESP_OK) {
        s_ws_send_pending = false;
        free(a);
    }
    return err;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, index_html_len);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);

    /* ESP-IDF: first /ws invocation is still HTTP_GET (handshake). Do not recv yet. */
    if (req->method == HTTP_GET) {
        ws_fd_set(fd);
        ESP_LOGI(TAG, "WebSocket client connected (fd=%d)", fd);
        return queue_quat_send(req->handle, fd);
    }

    httpd_ws_frame_t frame = {0};
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        ws_fd_clear_if(fd);
        return ESP_OK;
    }

    if (frame.type == HTTPD_WS_TYPE_PING) {
        httpd_ws_frame_t pong = {
            .final = true,
            .type = HTTPD_WS_TYPE_PONG,
            .payload = frame.payload,
            .len = frame.len,
        };
        (void)httpd_ws_send_frame(req, &pong);
        return ESP_OK;
    }

    if (frame.len == 0) {
        return ESP_OK;
    }

    uint8_t *buf = calloc(1, frame.len + 1);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }
    frame.payload = buf;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret == ESP_OK && frame.type == HTTPD_WS_TYPE_TEXT) {
        ws_fd_set(fd);
        if (strncmp((char *)buf, "reset", frame.len) == 0) {
            icm20948_request_calibration();
            imu_fusion_reset_reference();
            ESP_LOGI(TAG, "calibration requested + orientation reference reset");
        }
        queue_quat_send(req->handle, fd);
    }
    free(buf);
    return ret;
}

static void ws_broadcast_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(WS_BROADCAST_MS);

    while (1) {
        int fd = ws_fd_get();
        if (s_server && fd >= 0) {
            if (ws_client_alive(s_server, fd)) {
                queue_quat_send(s_server, fd);
            } else {
                ws_fd_clear_if(fd);
            }
        }
        vTaskDelay(period);
    }
}

esp_err_t web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.stack_size = 8192;
    config.close_fn = httpd_close_fn;
    config.lru_purge_enable = true;

    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config), TAG, "httpd start failed");

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .is_websocket = true,
    };

    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &root_uri), TAG, "root uri failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &ws_uri), TAG, "ws uri failed");

    BaseType_t ok = xTaskCreate(ws_broadcast_task, "ws_tx", 4096, NULL, 4, NULL);
    if (ok != pdPASS) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
