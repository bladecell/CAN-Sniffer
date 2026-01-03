#include "async_web_server.hpp"

static const char *TAG = "ASYNC_WEB_SERVER";

AsyncWebServer::AsyncWebServer() : server_(NULL), request_queue(NULL), worker_ready_count(NULL)
{
}

AsyncWebServer::~AsyncWebServer()
{
    stop();
}

esp_err_t AsyncWebServer::start(Config config)
{
    worker_handles.reserve(config.async_worker_task_num);
    start_workers(config.async_worker_task_num, config.async_worker_stack_size, config.async_worker_task_priority, config.worker_core_id);

    config.httpd_config.max_open_sockets = config.async_worker_task_num + 1;
    config.httpd_config.lru_purge_enable = true;

    ESP_RETURN_ON_ERROR(
        httpd_start(&server_, &config.httpd_config),
        TAG,
        "Failed to start server");

    ESP_LOGI(TAG, "Server started successfully.");

    return ESP_OK;
}

esp_err_t AsyncWebServer::stop()
{
    if (server_)
    {
        ESP_LOGI(TAG, "Stopping server...");
        esp_err_t err = httpd_stop(server_);
        server_ = NULL;
        return err;
    }

    for (TaskHandle_t worker_handle : worker_handles)
    {
        if (worker_handle != nullptr)
        {
            vTaskDelete(worker_handle);
        }
    }
    worker_handles.clear();

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

    for (RouteContext *ctx : route_contexts)
    {
        delete ctx;
    }
    route_contexts.clear();

    return ESP_OK;
}

void AsyncWebServer::registerRoute(const char *uri, httpd_method_t method, AsyncHandler func, void *arg)
{
    // 1. Allocate memory to hold the function pointer
    RouteContext *ctx = new RouteContext;
    ctx->handler = func;
    ctx->arg = arg;

    httpd_uri_t http_uri = {
        .uri = uri,
        .method = method,
        .handler = async_handler, // Point to our static shim (see below)
        .user_ctx = ctx           // Store our context here
    };

    route_contexts.push_back(ctx);

    // 2. Register with ESP-IDF
    esp_err_t err = httpd_register_uri_handler(server_, &http_uri);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register handler for %s", uri);
        delete ctx; // Clean up memory if registration fails
    }
}

esp_err_t AsyncWebServer::async_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "uri: %s", req->uri);

    // add to the async request queue
    if (AsyncWebServer::getInstance().queue_request(req) == ESP_OK)
    {
        return ESP_OK;
    }
    else
    {
        httpd_resp_set_status(req, "503 Busy");
        httpd_resp_sendstr(req, "<div> no workers available. server busy.</div>");
        return ESP_OK;
    }
}

esp_err_t AsyncWebServer::queue_request(httpd_req_t *req)
{

    RouteContext *ctx = (RouteContext *)req->user_ctx;

    if (!ctx || !ctx->handler)
    {
        ESP_LOGE(TAG, "No handler found in user_ctx");
        return ESP_FAIL;
    }

    // must create a copy of the request that we own
    httpd_req_t *copy = NULL;
    esp_err_t err = httpd_req_async_handler_begin(req, &copy);
    if (err != ESP_OK)
    {
        return err;
    }

    // create the job for the worker
    AsyncRequest job;
    job.req = copy;
    job.handler = ctx->handler;
    job.arg = ctx->arg;

    int ticks = 0;

    // counting semaphore: if success, we know 1 or
    // more asyncReqTaskWorkers are available.
    if (xSemaphoreTake(worker_ready_count, ticks) != pdTRUE)
    {
        ESP_LOGE(TAG, "No workers are available");
        httpd_req_async_handler_complete(copy); // cleanup
        return ESP_FAIL;
    }

    // Since worker_ready_count > 0 the queue should already have space.
    // But lets wait up to 100ms just to be safe.
    if (xQueueSend(request_queue, &job, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGE(TAG, "worker queue is full");
        httpd_req_async_handler_complete(copy);
        return ESP_FAIL;
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

        if (xQueueReceive(request_queue, &async_req, portMAX_DELAY))
        {
            if (async_req.handler && async_req.req)
            {
                // call the handler using the COPY
                async_req.handler(async_req.req, async_req.arg);

                // mark complete using the COPY
                // This cleans up the memory allocated by 'begin' and closes/frees the socket usage
                if (httpd_req_async_handler_complete(async_req.req) != ESP_OK)
                {
                    ESP_LOGE(TAG, "failed to complete async req");
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void AsyncWebServer::worker_task_wrapper(void *arg)
{
    AsyncWebServer *instance = (AsyncWebServer *)arg;
    instance->worker_task(); // Call instance method
}

// start worker threads
esp_err_t AsyncWebServer::start_workers(uint8_t num_workers, uint32_t stack_size, uint8_t priority, int core_id)
{

    // counting semaphore keeps track of available workers
    worker_ready_count = xSemaphoreCreateCounting(
        num_workers, // Max Count
        0);          // Initial Count
    if (worker_ready_count == NULL)
    {
        ESP_LOGE(TAG, "Failed to create workers counting Semaphore");
        return ESP_FAIL;
    }

    // create queue
    request_queue = xQueueCreate(1, sizeof(AsyncWebServer::AsyncRequest));
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

        BaseType_t res = xTaskCreatePinnedToCore(
            worker_task_wrapper, // Static wrapper
            "async_req_worker",
            stack_size,
            this, // Pass instance as argument
            priority,
            &hdl,
            core_id);

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