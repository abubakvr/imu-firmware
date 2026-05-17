#include "web_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_fusion.h"
#include "step_detect.h"
#include "index_html.h"

static const char *TAG = "web";
static httpd_handle_t s_server;
static int s_ws_fd = -1;

typedef struct {
    httpd_handle_t hd;
    int fd;
    char msg[128];
} ws_send_arg_t;

static void ws_send_work(void *arg)
{
    ws_send_arg_t *a = (ws_send_arg_t *)arg;
    httpd_ws_frame_t frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)a->msg,
        .len = strlen(a->msg),
    };
    if (httpd_ws_send_frame_async(a->hd, a->fd, &frame) != ESP_OK) {
        ESP_LOGW(TAG, "ws send failed fd=%d", a->fd);
    }
    free(a);
}

static esp_err_t queue_quat_send(httpd_handle_t hd, int fd)
{
    if (!hd || fd < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ws_send_arg_t *a = calloc(1, sizeof(*a));
    if (!a) {
        return ESP_ERR_NO_MEM;
    }

    float w, x, y, z;
    bool walking = step_detect_is_walking();
    uint32_t steps = step_detect_get_step_count();
    imu_fusion_get_display_quat(&w, &x, &y, &z);
    snprintf(a->msg, sizeof(a->msg),
             "{\"w\":%.4f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"walking\":%s,\"steps\":%lu}",
             w, x, y, z, walking ? "true" : "false", (unsigned long)steps);
    a->hd = hd;
    a->fd = fd;

    esp_err_t err = httpd_queue_work(hd, ws_send_work, a);
    if (err != ESP_OK) {
        free(a);
    }
    return err;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, index_html_len);
}

/* Called by ESP-IDF after the WebSocket handshake (uri->handler is NOT). */
static esp_err_t ws_post_handshake_cb(httpd_req_t *req)
{
    s_ws_fd = httpd_req_to_sockfd(req);
    ESP_LOGI(TAG, "WebSocket client connected (fd=%d)", s_ws_fd);
    return queue_quat_send(req->handle, s_ws_fd);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    s_ws_fd = httpd_req_to_sockfd(req);

    httpd_ws_frame_t frame = {0};
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        return ret;
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
        if (strncmp((char *)buf, "reset", frame.len) == 0) {
            imu_fusion_reset_reference();
            ESP_LOGI(TAG, "orientation reference reset");
        }
    }
    free(buf);
    return ret;
}

static void ws_broadcast_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(20);

    while (1) {
        if (s_server && s_ws_fd >= 0) {
            queue_quat_send(s_server, s_ws_fd);
        }
        vTaskDelay(period);
    }
}

esp_err_t web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.stack_size = 8192;

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
        .ws_post_handshake_cb = ws_post_handshake_cb,
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
