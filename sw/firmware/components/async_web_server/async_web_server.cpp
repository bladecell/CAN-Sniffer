#include "async_web_server.hpp"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "ASYNC_WEB_SERVER";

struct WsSendContext
{
    httpd_handle_t server;
    int            fd;
    size_t         len;
    uint8_t        payload[0];
};

static void ws_send_work_func(void* arg)
{
    WsSendContext* ctx = (WsSendContext*)arg;

    httpd_ws_frame_t ws_frame = {};
    ws_frame.final            = true;
    ws_frame.fragmented       = false;
    ws_frame.type             = HTTPD_WS_TYPE_BINARY;
    ws_frame.payload          = ctx->payload;
    ws_frame.len              = ctx->len;

    httpd_ws_send_frame_async(ctx->server, ctx->fd, &ws_frame);

    free(ctx);
}

AsyncWebServer::AsyncWebServer() : server_(NULL), request_queue(NULL)
{
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
    config.httpd_config.lru_purge_enable = false;
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
        xQueueReset(request_queue);
        AsyncRequest poison = {};
        for (size_t i = 0; i < worker_handles.size(); i++)
            xQueueSend(request_queue, &poison, pdMS_TO_TICKS(100));
    }

    if (shutdown_sem_)
    {
        for (size_t i = 0; i < worker_handles.size(); i++)
            xSemaphoreTake(shutdown_sem_, pdMS_TO_TICKS(500));
    }

    for (auto& w : worker_handles)
    {
        if (w.task)
            vTaskDelete(w.task);
        heap_caps_free(w.tcb);
        heap_caps_free(w.stack);
    }
    worker_handles.clear();

    if (shutdown_sem_)
    {
        vSemaphoreDelete(shutdown_sem_);
        shutdown_sem_ = NULL;
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

    esp_err_t err = httpd_register_uri_handler(server_, &http_uri);
    if (err != ESP_OK)
    {
        delete ctx;
        ESP_LOGE(TAG, "Failed to register handler for %s", uri);
        return;
    }
    route_contexts.push_back(ctx);
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
    httpd_req_t* copy = NULL;

    // 1. Detach the request first
    if (httpd_req_async_handler_begin(req, &copy) != ESP_OK)
        return ESP_FAIL;

    RouteContext* ctx = (RouteContext*)req->user_ctx;

    AsyncRequest job;
    job.req           = copy;
    job.handler       = ctx->handler;
    job.arg           = ctx->arg;
    job.start_time_us = esp_timer_get_time();

    // 2. Let the FreeRTOS Queue act as the gatekeeper
    if (xQueueSend(request_queue, &job, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Server busy, internal queue full.");
        httpd_resp_set_status(copy, "503 Busy");
        httpd_resp_sendstr(copy, "{\"error\":\"Server busy.\"}");
        httpd_req_async_handler_complete(copy);
        return ESP_OK;
    }

    return ESP_OK;
}

// each worker thread loops forever, processing requests
void AsyncWebServer::worker_task()
{
    AsyncRequest async_req;

    while (running_)
    {
        if (xQueueReceive(request_queue, &async_req, portMAX_DELAY) != pdTRUE)
            continue;

        if (!async_req.handler && !async_req.req)
            break;  // poison pill — exit loop

        if (async_req.handler && async_req.req)
        {
            char uri_buf[128];
            strlcpy(uri_buf, async_req.req->uri, sizeof(uri_buf));

            async_req.handler(async_req.req, async_req.arg);

            int64_t duration_us = esp_timer_get_time() - async_req.start_time_us;
            ESP_LOGI(TAG, "%s processed in %.2f ms", uri_buf, duration_us / 1000.0f);

            if (async_req.req->aux != NULL)
            {
                esp_err_t err = httpd_req_async_handler_complete(async_req.req);
                if (err != ESP_OK)
                    ESP_LOGE(TAG, "failed to complete async req: %s", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGW(TAG, "Request already closed, skipping complete.");
            }
        }
    }

    xSemaphoreGive(shutdown_sem_);
    vTaskSuspend(NULL);
}

void AsyncWebServer::worker_task_wrapper(void* arg)
{
    AsyncWebServer* instance = (AsyncWebServer*)arg;
    instance->worker_task();  // Call instance method
}

// start worker threads
esp_err_t AsyncWebServer::start_workers(uint8_t num_workers, uint32_t stack_size, uint8_t priority, int core_id)
{
    // create queue
    uint8_t queue_depth = num_workers * 2;
    request_queue       = xQueueCreate(queue_depth, sizeof(AsyncWebServer::AsyncRequest));
    if (request_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create request_queue");
        return ESP_FAIL;
    }

    // start worker tasks
    for (int i = 0; i < num_workers; i++)
    {
        StaticTask_t* task_buf =
            (StaticTask_t*)heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

        StackType_t* stack =
            (StackType_t*)heap_caps_malloc(stack_size * sizeof(StackType_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

        if (!task_buf || !stack)
        {
            if (task_buf)
                heap_caps_free(task_buf);
            if (stack)
                heap_caps_free(stack);
            ESP_LOGE(TAG, "Failed to allocate memory for worker %d. Stack size too large?", i);
            continue;
        }

        TaskHandle_t hdl = xTaskCreateStaticPinnedToCore(worker_task_wrapper, "async_req_worker", stack_size, this,
                                                         priority, stack, task_buf, core_id);

        if (hdl == NULL)
        {
            heap_caps_free(task_buf);
            heap_caps_free(stack);
            ESP_LOGE(TAG, "Failed to start async worker %d", i);
            continue;
        }

        worker_handles.push_back({hdl, task_buf, stack});
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

    static const size_t MAX_CLIENTS = 20;
    int                 client_fds[MAX_CLIENTS];
    size_t              fds = MAX_CLIENTS;

    if (httpd_get_client_list(server_, &fds, client_fds) == ESP_OK)
    {
        for (size_t i = 0; i < fds; i++)
        {
            if (httpd_ws_get_fd_info(server_, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET)
            {
                // 1. Dynamically allocate memory: size of struct + size of payload
                WsSendContext* ctx =
                    (WsSendContext*)heap_caps_malloc(sizeof(WsSendContext) + ws_pkt->len, MALLOC_CAP_SPIRAM);

                if (!ctx)
                {
                    ESP_LOGW(TAG, "Failed to allocate memory for WS broadcast");
                    continue;  // Skip this client if out of memory
                }

                // 2. Populate the context and copy the payload immediately
                ctx->server = server_;
                ctx->fd     = client_fds[i];
                ctx->len    = ws_pkt->len;
                memcpy(ctx->payload, ws_pkt->payload, ws_pkt->len);

                // 3. Queue the work to the HTTP task
                esp_err_t err = httpd_queue_work(server_, ws_send_work_func, ctx);

                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to queue WS work");
                    free(ctx);  // Prevent memory leak if queueing fails
                }
            }
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
