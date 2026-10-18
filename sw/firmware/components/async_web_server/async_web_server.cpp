#include <unistd.h>

#include <cstdio>

#include <new>
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

esp_err_t AsyncWebServer::sendError(httpd_req_t* req, int status, const char* reason, const char* code,
                                    uint32_t retryAfterS)
{
    status = http_error::errorStatus(status);
    char body[http_error::kBodyMax];
    if (http_error::buildBody(body, sizeof(body), status, reason, code) == 0)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err = httpd_resp_set_status(req, http_error::statusLine(status));
    if (err == ESP_OK)
    {
        err = httpd_resp_set_type(req, "application/json");
    }
    if (err == ESP_OK)
    {
        err = httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    }
    char retryBuf[12];
    if (retryAfterS == 0)
    {
        retryAfterS = http_error::retryAfterS(status);
    }
    if (err == ESP_OK && retryAfterS != 0)
    {
        (void)snprintf(retryBuf, sizeof(retryBuf), "%u", static_cast<unsigned>(retryAfterS));
        err = httpd_resp_set_hdr(req, "Retry-After", retryBuf);  // retryBuf lives until the send below
    }
    return err == ESP_OK ? httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN) : err;
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
    running_        = true;
    stoppedWorkers_ = 0;
    poisonsQueued_  = 0;

    shutdown_sem_ = xSemaphoreCreateCounting(config.async_worker_task_num, 0);
    if (!shutdown_sem_)
    {
        running_ = false;
        return ESP_ERR_NO_MEM;
    }

    worker_handles.reserve(config.async_worker_task_num);
    esp_err_t err = start_workers(config.async_worker_task_num, config.async_worker_stack_size,
                                  config.async_worker_task_priority, config.worker_core_id);
    if (err == ESP_OK && worker_handles.empty())
    {
        err = ESP_ERR_NO_MEM;
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start async workers: %s", esp_err_to_name(err));
        stop();  // frees the queue, semaphore and any worker that did start
        return err;
    }

    config.httpd_config.max_open_sockets = config.max_open_sockets;
    config.httpd_config.lru_purge_enable = false;
    config.httpd_config.stack_size       = 8192;

    max_requests_per_sec_ = config.max_requests_per_sec;
    socketClosedHook_     = config.on_socket_closed;
    requestFilter_        = config.request_filter;
    config.httpd_config.close_fn = &AsyncWebServer::closeSocket;

    server_ = NULL;
    err     = httpd_start(&server_, &config.httpd_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start server: %s", esp_err_to_name(err));
        server_ = NULL;
        stop();
        return err;
    }
    ESP_LOGI(TAG, "Server started successfully.");
    return ESP_OK;
}

// httpd close_fn replaces the default close(), so it must close the socket itself.
void AsyncWebServer::closeSocket(httpd_handle_t, int sockfd)
{
    close(sockfd);
    AsyncWebServer& self = getInstance();
    if (self.socketClosedHook_ != nullptr)
    {
        self.socketClosedHook_(sockfd);
    }
}

esp_err_t AsyncWebServer::stop()
{
    running_ = false;

    // Drain workers FIRST, while the server is still alive, so their request
    // data stays valid. Each worker exits after the poison pill (or when the
    // current request finishes) and gives shutdown_sem_ exactly once.
    // running_ stays false after a timed-out stop on purpose: the server is draining, new requests get 503 and the
    // supervisor either retries stop() or restarts. One pill per worker over all attempts, so a retry queues none twice.
    if (request_queue)
    {
        AsyncRequest poison = {};
        while (poisonsQueued_ < worker_handles.size() &&
               xQueueSend(request_queue, &poison, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            poisonsQueued_++;
        }
    }

    // Bounded: a worker stuck in a handler must not hang the caller (the supervisor retries and caps its attempts).
    // Nothing is freed on timeout; calling stop() again waits for the workers that are still running.
    if (shutdown_sem_)
    {
        const int64_t deadline = esp_timer_get_time() + STOP_WAIT_US;
        while (stoppedWorkers_ < worker_handles.size())
        {
            const int64_t left = deadline - esp_timer_get_time();
            if (left <= 0 || xSemaphoreTake(shutdown_sem_, pdMS_TO_TICKS(left / 1000)) != pdTRUE)
            {
                ESP_LOGE(TAG, "%u of %u async workers still busy, server not stopped",
                         static_cast<unsigned>(worker_handles.size() - stoppedWorkers_),
                         static_cast<unsigned>(worker_handles.size()));
                return ESP_ERR_TIMEOUT;
            }
            stoppedWorkers_++;
        }
    }

    // Workers leave on the poison pill or when running_ drops, so requests may still sit in the queue (ASYNC-2).
    // Answer each with 503 and release it, while httpd is still alive.
    drainQueue();

    // Now safe to stop the httpd server: no worker is touching its data.
    if (server_)
    {
        httpd_stop(server_);
        server_ = NULL;
    }

    worker_handles.clear();
    stoppedWorkers_ = 0;
    poisonsQueued_  = 0;

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
    {
        delete ctx;
    }
    route_contexts.clear();

    return ESP_OK;
}

// Caller guarantees no worker is running. Poison pills have no request and are discarded.
void AsyncWebServer::drainQueue()
{
    if (!request_queue)
    {
        return;
    }
    AsyncRequest job;
    while (xQueueReceive(request_queue, &job, 0) == pdTRUE)
    {
        if (job.req == nullptr)
        {
            continue;
        }
        sendError(job.req, 503, "Server is shutting down", http_error::kShuttingDown);
        httpd_req_async_handler_complete(job.req);
    }
}

esp_err_t AsyncWebServer::registerRoute(const char* uri, httpd_method_t method, AsyncHandler func, void* arg)
{
    // 1. Allocate memory to hold the function pointer
    RouteContext* ctx = new (std::nothrow) RouteContext;
    if (!ctx)
    {
        ESP_LOGE(TAG, "Out of memory registering %s", uri);
        return ESP_ERR_NO_MEM;
    }
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
        return err;
    }
    route_contexts.push_back(ctx);
    return ESP_OK;
}

esp_err_t AsyncWebServer::async_handler(httpd_req_t* req)
{
    ESP_LOGD(TAG, "%s uri: %s", get_method_str(req->method), req->uri);

    AsyncWebServer& self = AsyncWebServer::getInstance();
    if (self.requestFilter_ != nullptr && !self.requestFilter_(req, static_cast<RouteContext*>(req->user_ctx)->arg))
    {
        return ESP_OK;  // the filter answered
    }

    // add to the async request queue
    if (self.queue_request(req) == ESP_OK)
    {
        return ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG, "Server overload, dropping request to %s", req->uri);
        sendError(req, 503, "Server busy, no background workers available", http_error::kBusy);
        return ESP_OK;
    }
}

esp_err_t AsyncWebServer::queue_request(httpd_req_t* req)
{
    if (!running_)
    {
        // Server is stopping; reject new work synchronously instead of detaching it.
        sendError(req, 503, "Server is shutting down", http_error::kShuttingDown);
        return ESP_OK;
    }

    httpd_req_t* copy = NULL;

    // 1. Detach the request first
    if (httpd_req_async_handler_begin(req, &copy) != ESP_OK)
    {
        return ESP_FAIL;
    }

    int64_t now = esp_timer_get_time();
    if (now - window_start_us_ >= 1000000)
    {
        window_start_us_       = now;
        req_count_this_window_ = 0;
    }
    req_count_this_window_++;

    if (req_count_this_window_ > max_requests_per_sec_)
    {
        ESP_LOGW(TAG, "Rate limit exceeded, uri=%s", copy->uri);
        sendError(copy, 429, "Rate limit exceeded", http_error::kRateLimited);
        httpd_req_async_handler_complete(copy);
        return ESP_OK;
    }

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
        sendError(copy, 503, "Server busy", http_error::kBusy);
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
        {
            continue;
        }

        if (!async_req.handler && !async_req.req)
        {
            break;  // poison pill — exit loop
        }

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
                {
                    ESP_LOGE(TAG, "failed to complete async req: %s", esp_err_to_name(err));
                }
            }
            else
            {
                ESP_LOGW(TAG, "Request already closed, skipping complete.");
            }
        }
    }

    xSemaphoreGive(shutdown_sem_);
    vTaskDelete(NULL);
}

void AsyncWebServer::worker_task_wrapper(void* arg)
{
    AsyncWebServer* instance = (AsyncWebServer*)arg;
    instance->worker_task();
    vTaskDelete(NULL);  // defensive: worker_task() should already self-delete
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
        TaskHandle_t hdl = NULL;

        BaseType_t created =
            xTaskCreatePinnedToCore(worker_task_wrapper, "async_req_worker", stack_size, this, priority, &hdl, core_id);

        if (created != pdPASS || hdl == NULL)
        {
            ESP_LOGE(TAG, "Failed to start async worker %d", i);
            continue;
        }

        worker_handles.push_back({hdl});
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

esp_err_t AsyncWebServer::registerSocketRoute(const char* uri, esp_err_t (*handler)(httpd_req_t* r), void* ctx)
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
    return err;
}

void AsyncWebServer::wsBroadcast(httpd_ws_frame_t* ws_pkt)
{
    if (!server_ || !ws_pkt || !ws_pkt->payload || ws_pkt->len == 0)
    {
        return;
    }

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
                    (WsSendContext*)heap_caps_malloc(sizeof(WsSendContext) + ws_pkt->len, MALLOC_CAP_INTERNAL);

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
                    heap_caps_free(ctx);  // Prevent memory leak if queueing fails
                }
            }
        }
    }
}

uint32_t AsyncWebServer::getActiveWSClientCount(int excludeFd)
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
        if (fd == excludeFd)
        {
            continue;
        }

        httpd_ws_client_info_t info = httpd_ws_get_fd_info(server_, fd);

        if (info == HTTPD_WS_CLIENT_WEBSOCKET)
        {
            client_count++;
        }
    }

    return client_count;
}
