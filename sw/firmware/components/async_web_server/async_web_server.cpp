#include "async_web_server.hpp"

#include <atomic>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "ASYNC_WEB_SERVER";

AsyncWebServer::AsyncWebServer() : server_(NULL), request_queue(NULL), worker_ready_count(NULL)
{
    ws_mutex = xSemaphoreCreateMutex();
}

AsyncWebServer::~AsyncWebServer()
{
    stop();
}

esp_err_t AsyncWebServer::start(Config config)
{
    running_ = true;

    shutdown_sem_ = xSemaphoreCreateCounting(config.async_worker_task_num, 0);
    if (!shutdown_sem_)
        return ESP_FAIL;

    worker_handles.reserve(config.async_worker_task_num);
    start_workers(config.async_worker_task_num, config.async_worker_stack_size, config.async_worker_task_priority,
                  config.worker_core_id);

    config.httpd_config.max_open_sockets = config.max_open_sockets;
    config.httpd_config.lru_purge_enable = true;
    config.httpd_config.stack_size       = 8192;

    ESP_RETURN_ON_ERROR(httpd_start(&server_, &config.httpd_config), TAG, "Failed to start server");
    ESP_LOGI(TAG, "Server started successfully.");
    return ESP_OK;
}

esp_err_t AsyncWebServer::stop()
{
    if (server_)
    {
        httpd_stop(server_);
        server_ = NULL;
    }

    running_ = false;

    if (request_queue)
    {
        AsyncRequest poison = {};
        for (size_t i = 0; i < worker_handles.size(); i++)
            xQueueSend(request_queue, &poison, pdMS_TO_TICKS(100));
    }

    if (shutdown_sem_)
    {
        for (size_t i = 0; i < worker_handles.size(); i++)
            xSemaphoreTake(shutdown_sem_, pdMS_TO_TICKS(500));
    }

    for (TaskHandle_t h : worker_handles)
        if (h)
            vTaskDelete(h);
    worker_handles.clear();

    if (shutdown_sem_)
    {
        vSemaphoreDelete(shutdown_sem_);
        shutdown_sem_ = NULL;
    }
    if (worker_ready_count)
    {
        vSemaphoreDelete(worker_ready_count);
        worker_ready_count = NULL;
    }
    if (request_queue)
    {
        vQueueDelete(request_queue);
        request_queue = NULL;
    }

    for (RouteContext* ctx : route_contexts)
        delete ctx;
    route_contexts.clear();

    return ESP_OK;
}

void AsyncWebServer::registerRoute(const char* uri, httpd_method_t method, AsyncHandler func, void* arg)
{
    // 1. Allocate memory to hold the function pointer
    RouteContext* ctx = new RouteContext;
    ctx->handler      = func;
    ctx->arg          = arg;

    httpd_uri_t http_uri = {};
    http_uri.uri         = uri;
    http_uri.method      = method;
    http_uri.handler     = async_handler;  // Point to our static shim (see below)
    http_uri.user_ctx    = ctx;            // Store our context here

    route_contexts.push_back(ctx);

    // 2. Register with ESP-IDF
    esp_err_t err = httpd_register_uri_handler(server_, &http_uri);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register handler for %s", uri);
        delete ctx;  // Clean up memory if registration fails
    }
}

esp_err_t AsyncWebServer::async_handler(httpd_req_t* req)
{
    ESP_LOGD(TAG, "%s uri: %s", get_method_str(req->method), req->uri);

    // add to the async request queue
    if (AsyncWebServer::getInstance().queue_request(req) == ESP_OK)
    {
        return ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG, "Server overload, dropping request to %s", req->uri);
        httpd_resp_set_status(req, "503 Busy");
        httpd_resp_sendstr(req, "{\"error\": \"Server busy, no background workers available.\"}");
        return ESP_OK;
    }
}

esp_err_t AsyncWebServer::queue_request(httpd_req_t* req)
{
    RouteContext* ctx = (RouteContext*)req->user_ctx;

    if (!ctx || !ctx->handler)
    {
        ESP_LOGE(TAG, "No handler found in user_ctx");
        return ESP_FAIL;
    }

    // Check for available workers BEFORE we shred the request lifecycle!
    if (xSemaphoreTake(worker_ready_count, 0) != pdTRUE)
    {
        // Returning ESP_FAIL here leaves 'req' perfectly safe for async_handler to use.
        return ESP_FAIL;
    }

    // We safely decouple the request from the HTTP server
    httpd_req_t* copy = NULL;
    esp_err_t    err  = httpd_req_async_handler_begin(req, &copy);
    if (err != ESP_OK)
    {
        xSemaphoreGive(worker_ready_count);
        return ESP_FAIL;
    }

    // create the job for the worker
    AsyncRequest job;
    job.req           = copy;
    job.handler       = ctx->handler;
    job.arg           = ctx->arg;
    job.start_time_us = esp_timer_get_time();

    if (xQueueSend(request_queue, &job, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "Critical Fault: Queue full despite available worker!");

        // We already detached the request into 'copy'. We CANNOT return ESP_FAIL
        // because async_handler would try to use the dead 'req' pointer.
        // Instead, we answer using our valid 'copy', complete it, and return ESP_OK.
        httpd_resp_set_status(copy, "503 Busy");
        httpd_resp_sendstr(copy, "{\"error\": \"Server busy, internal queue full.\"}");
        httpd_req_async_handler_complete(copy);

        xSemaphoreGive(worker_ready_count);

        return ESP_OK;  // Return OK since we sent the 503 status code
    }

    return ESP_OK;
}

// each worker thread loops forever, processing requests
void AsyncWebServer::worker_task()
{
    AsyncRequest async_req;

    while (true)
    {
        xSemaphoreGive(worker_ready_count);

        if (xQueueReceive(request_queue, &async_req, portMAX_DELAY) != pdTRUE)
            continue;

        if (!running_ || (!async_req.handler && !async_req.req))
        {
            xSemaphoreGive(shutdown_sem_);
            vTaskDelete(NULL);
            return;
        }

        if (async_req.handler && async_req.req)
        {
            async_req.handler(async_req.req, async_req.arg);

            bool already_closed = (async_req.req->aux == NULL);
            if (!already_closed)
            {
                int64_t duration_us = esp_timer_get_time() - async_req.start_time_us;
                ESP_LOGI(TAG, "%s processed in %.2f ms", async_req.req->uri, duration_us / 1000.0f);

                esp_err_t err = httpd_req_async_handler_complete(async_req.req);
                if (err != ESP_OK)
                    ESP_LOGE(TAG, "failed to complete async req: %s", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGW(TAG, "Request already closed by handler, skipping complete.");
            }
        }
    }
}

void AsyncWebServer::worker_task_wrapper(void* arg)
{
    AsyncWebServer* instance = (AsyncWebServer*)arg;
    instance->worker_task();  // Call instance method
}

// start worker threads
esp_err_t AsyncWebServer::start_workers(uint8_t num_workers, uint32_t stack_size, uint8_t priority, int core_id)
{
    // counting semaphore keeps track of available workers
    worker_ready_count = xSemaphoreCreateCounting(num_workers,  // Max Count
                                                  0);           // Initial Count
    if (worker_ready_count == NULL)
    {
        ESP_LOGE(TAG, "Failed to create workers counting Semaphore");
        return ESP_FAIL;
    }

    // create queue
    request_queue = xQueueCreate(num_workers, sizeof(AsyncWebServer::AsyncRequest));
    if (request_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create request_queue");
        vSemaphoreDelete(worker_ready_count);
        return ESP_FAIL;
    }

    // start worker tasks
    for (int i = 0; i < num_workers; i++)
    {
        TaskHandle_t hdl = NULL;

        BaseType_t res = xTaskCreatePinnedToCore(worker_task_wrapper,  // Static wrapper
                                                 "async_req_worker", stack_size,
                                                 this,  // Pass instance as argument
                                                 priority, &hdl, core_id);

        if (res == pdPASS)
        {
            worker_handles.push_back(hdl);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to start async worker %d", i);
        }
    }

    return ESP_OK;
}

const char* AsyncWebServer::get_method_str(int method)
{
    switch (method)
    {
        case HTTP_GET:
            return "GET";
        case HTTP_POST:
            return "POST";
        case HTTP_PUT:
            return "PUT";
        case HTTP_DELETE:
            return "DELETE";
        case HTTP_HEAD:
            return "HEAD";
        case HTTP_PATCH:
            return "PATCH";
        default:
            return "UNKNOWN";
    }
}

void AsyncWebServer::registerSocketRoute(const char* uri, esp_err_t (*handler)(httpd_req_t* r), void* ctx)
{
    httpd_uri_t http_uri = {};

    http_uri.uri                      = uri;
    http_uri.method                   = HTTP_GET;
    http_uri.handler                  = handler;
    http_uri.user_ctx                 = ctx;
    http_uri.is_websocket             = true;
    http_uri.handle_ws_control_frames = true;

    esp_err_t err = httpd_register_uri_handler(server_, &http_uri);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register WS handler for %s", uri);
    }
}

void AsyncWebServer::wsBroadcast(httpd_ws_frame_t* ws_pkt)
{
    if (!server_ || !ws_pkt || !ws_pkt->payload || ws_pkt->len == 0)
        return;

    static struct
    {
        uint8_t          payload[64];
        httpd_ws_frame_t frame;
    } vault[16];

    static uint8_t vault_idx = 0;

    // Protect the slot grab + memcpy together — atomic fetch_add alone
    // doesn't prevent two tasks from racing into the same slot's memcpy
    if (xSemaphoreTake(ws_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return;

    uint8_t slot = vault_idx++ % 16;
    auto&   v    = vault[slot];

    size_t len = (ws_pkt->len > 64) ? 64 : ws_pkt->len;
    memcpy(v.payload, ws_pkt->payload, len);
    v.frame.type    = ws_pkt->type;
    v.frame.payload = v.payload;
    v.frame.len     = len;
    v.frame.final   = true;

    xSemaphoreGive(ws_mutex);

    // Send outside the mutex — httpd_ws_send_frame_async is thread-safe
    static const size_t MAX_CLIENTS = 20;
    int                 client_fds[MAX_CLIENTS];
    size_t              fds = MAX_CLIENTS;

    if (httpd_get_client_list(server_, &fds, client_fds) == ESP_OK)
    {
        for (size_t i = 0; i < fds; i++)
        {
            if (httpd_ws_get_fd_info(server_, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET)
                httpd_ws_send_frame_async(server_, client_fds[i], &v.frame);
        }
    }
}

uint32_t AsyncWebServer::getActiveWSClientCount()
{
    static const size_t MAX_CLIENTS = 20;
    int                 client_fds[MAX_CLIENTS];
    size_t              fds = MAX_CLIENTS;

    esp_err_t ret = httpd_get_client_list(server_, &fds, client_fds);

    if (ret != ESP_OK)
    {
        return 0;
    }

    uint32_t client_count = 0;

    for (int i = 0; i < fds; i++)
    {
        int fd = client_fds[i];

        httpd_ws_client_info_t info = httpd_ws_get_fd_info(server_, fd);

        if (info == HTTPD_WS_CLIENT_WEBSOCKET)
        {
            client_count++;
        }
    }

    return client_count;
}
