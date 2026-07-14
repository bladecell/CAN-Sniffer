#pragma once

#include <cstdint>
#include <vector>

#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define MAX_WS_CLIENTS CONFIG_LWIP_MAX_LISTENING_TCP

class AsyncWebServer
{
public:
    AsyncWebServer();
    ~AsyncWebServer();

    struct Config
    {
        httpd_config_t httpd_config               = HTTPD_DEFAULT_CONFIG();
        uint8_t        async_worker_task_num      = 5;
        uint8_t        max_open_sockets           = 5;
        uint8_t        async_worker_task_priority = 5;
        uint32_t       async_worker_stack_size    = 8192;
        int            worker_core_id             = tskNO_AFFINITY;
        uint32_t       max_requests_per_sec       = 50;
    };

    static AsyncWebServer& getInstance()
    {
        static AsyncWebServer instance;
        return instance;
    }

    using AsyncHandler = esp_err_t (*)(httpd_req_t* req, void* arg);

    struct AsyncRequest
    {
        AsyncHandler handler;
        httpd_req_t* req;
        void*        arg;
        int64_t      start_time_us;
    };

    struct RouteContext
    {
        AsyncHandler handler;
        void*        arg;
    };

    static const char* get_method_str(int method);

    esp_err_t        queue_request(httpd_req_t* req);
    static esp_err_t async_handler(httpd_req_t* req);

    esp_err_t start(Config config);
    esp_err_t stop();
    void      registerRoute(const char* uri, httpd_method_t method, AsyncHandler func, void* arg);

    void registerSocketRoute(const char* uri, esp_err_t (*handler)(httpd_req_t* r), void* ctx = NULL);

    void wsBroadcast(httpd_ws_frame_t* ws_pkt);

    uint32_t getActiveWSClientCount();

    struct WorkerHandle
    {
        TaskHandle_t  task;
        StaticTask_t* tcb;
        StackType_t*  stack;
    };

private:
    AsyncWebServer(const AsyncWebServer&)            = delete;
    AsyncWebServer& operator=(const AsyncWebServer&) = delete;

    httpd_handle_t server_;

    volatile bool     running_      = false;
    SemaphoreHandle_t shutdown_sem_ = NULL;

    // Async requests are queued here while they wait to
    // be processed by the workers
    QueueHandle_t request_queue;
    // Each worker has its own thread
    std::vector<WorkerHandle>  worker_handles;
    std::vector<RouteContext*> route_contexts;

    esp_err_t   start_workers(uint8_t num_workers, uint32_t stack_size, uint8_t priority, int core_id);
    void        worker_task();
    static void worker_task_wrapper(void* arg);

    uint32_t max_requests_per_sec_  = 50;
    uint32_t req_count_this_window_ = 0;
    int64_t  window_start_us_       = 0;
};