#pragma once

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <vector>

class AsyncWebServer
{
public:
    AsyncWebServer();
    ~AsyncWebServer();

    struct Config
    {
        httpd_config_t httpd_config = HTTPD_DEFAULT_CONFIG();
        uint8_t async_worker_task_num = 2;
        uint8_t async_worker_task_priority = 5;
        uint32_t async_worker_stack_size = 8192;
        int worker_core_id = tskNO_AFFINITY;
    };

    static AsyncWebServer &getInstance()
    {
        static AsyncWebServer instance;
        return instance;
    }

    using AsyncHandler = esp_err_t (*)(httpd_req_t *req, void *arg);

    struct AsyncRequest
    {
        AsyncHandler handler;
        httpd_req_t *req;
        void *arg;
    };

    struct RouteContext
    {
        AsyncHandler handler;
        void *arg;
    };

    static inline const char *get_method_str(int method);

    esp_err_t queue_request(httpd_req_t *req);
    static esp_err_t async_handler(httpd_req_t *req);

    esp_err_t start(Config config);
    esp_err_t stop();
    void registerRoute(const char *uri, httpd_method_t method, AsyncHandler func, void *arg);

private:
    AsyncWebServer(const AsyncWebServer &) = delete;
    AsyncWebServer &operator=(const AsyncWebServer &) = delete;

    httpd_handle_t server_;

    // Async requests are queued here while they wait to
    // be processed by the workers
    QueueHandle_t request_queue;
    // Track the number of free workers at any given time
    SemaphoreHandle_t worker_ready_count;
    // Each worker has its own thread
    std::vector<TaskHandle_t> worker_handles;
    std::vector<RouteContext *> route_contexts;

    esp_err_t start_workers(uint8_t num_workers, uint32_t stack_size, uint8_t priority, int core_id);
    void worker_task();
    static void worker_task_wrapper(void *arg);
};