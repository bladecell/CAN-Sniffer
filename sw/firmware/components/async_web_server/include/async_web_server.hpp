#pragma once

#include <cstdint>
#include <vector>

#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "http_error.hpp"

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
        // Called from the httpd task for every socket that closes, clean or not (peer reset, timeout, LRU purge), after
        // the socket is closed. Lets callers drop per-connection state. Must not block.
        void (*on_socket_closed)(int fd) = nullptr;
        // Runs in the httpd task for every REST request, before the rate limit and before the request is queued.
        // `routeArg` is the arg given to registerRoute(). Return false after answering the request itself (sendError);
        // the request is then dropped. Must not block: it delays every socket.
        bool (*request_filter)(httpd_req_t* req, void* routeArg) = nullptr;
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

    // The one way to answer an error: valid reason phrase, Content-Type application/json, body
    // {"status":"error","reason","code"} (http_error.hpp), Retry-After for 429/503. A null reason or code uses the
    // status default; retryAfterS 0 uses the status default too. Works on detached async requests as well.
    static esp_err_t sendError(httpd_req_t* req, int status, const char* reason = nullptr, const char* code = nullptr,
                               uint32_t retryAfterS = 0);

    esp_err_t        queue_request(httpd_req_t* req);
    static esp_err_t async_handler(httpd_req_t* req);

    esp_err_t start(Config config);
    // Waits at most STOP_WAIT_US for the workers. ESP_ERR_TIMEOUT: a worker is still in a handler, nothing was freed and
    // the server keeps running; call again later.
    esp_err_t stop();
    // Registration failures (no memory, httpd table full) are returned so start-up can fail instead of booting with a hole.
    esp_err_t registerRoute(const char* uri, httpd_method_t method, AsyncHandler func, void* arg);

    esp_err_t registerSocketRoute(const char* uri, esp_err_t (*handler)(httpd_req_t* r), void* ctx = NULL);

    void wsBroadcast(httpd_ws_frame_t* ws_pkt);

    // WebSocket clients currently connected, not counting `excludeFd` (the socket being closed, still listed by httpd).
    uint32_t getActiveWSClientCount(int excludeFd = -1);

    struct WorkerHandle
    {
        TaskHandle_t task;
    };

private:
    AsyncWebServer(const AsyncWebServer&)            = delete;
    AsyncWebServer& operator=(const AsyncWebServer&) = delete;

    httpd_handle_t server_;

    static constexpr int64_t STOP_WAIT_US = 3LL * 1000 * 1000;

    volatile bool     running_      = false;
    size_t            poisonsQueued_  = 0;  // poison pills queued since start(), at most one per worker
    size_t            stoppedWorkers_ = 0;  // workers that gave shutdown_sem_ since start()
    SemaphoreHandle_t shutdown_sem_ = NULL;

    // Async requests are queued here while they wait to
    // be processed by the workers
    QueueHandle_t request_queue;
    // Each worker has its own thread
    std::vector<WorkerHandle>  worker_handles;
    std::vector<RouteContext*> route_contexts;

    esp_err_t   start_workers(uint8_t num_workers, uint32_t stack_size, uint8_t priority, int core_id);
    void        drainQueue();
    void        worker_task();
    static void worker_task_wrapper(void* arg);

    void (*socketClosedHook_)(int fd) = nullptr;
    bool (*requestFilter_)(httpd_req_t* req, void* routeArg) = nullptr;
    static void closeSocket(httpd_handle_t hd, int sockfd);

    uint32_t max_requests_per_sec_  = 50;
    uint32_t req_count_this_window_ = 0;
    int64_t  window_start_us_       = 0;
};
