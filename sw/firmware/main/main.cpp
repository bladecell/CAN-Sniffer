
#include "esp_log.h"
#include "wifi.hpp"
#include <string>
#include "esp_http_server.h"
#include "web_assets.h"
#include "mdns.h"

static const char *TAG = "APP_MAIN";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    // Check the boolean generated in your header
    if (INDEX_HTML_IS_GZ)
    {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    }

    // Use the names that exist in your web_assets.h
    return httpd_resp_send(req, (const char *)INDEX_HTML, INDEX_HTML_LEN);
}

static esp_err_t svg_handler(httpd_req_t *req)
{
    // 1. Set the correct MIME type for an SVG
    httpd_resp_set_type(req, "image/svg+xml");

    // 2. If your Python script gzipped the SVG, add this header
    // Check your web_assets.h to see if SVELTE_SVG_IS_GZ is true
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    // 3. Send the data from your header file
    return httpd_resp_send(req, (const char *)SVELTE_SVG, SVELTE_SVG_LEN);
}

static const httpd_uri_t hello_world_uri = {
    .uri = "/",               // the address at which the resource can be found
    .method = HTTP_GET,       // The HTTP method (HTTP_GET, HTTP_POST, ...)
    .handler = index_handler, // The function which process the request
    .user_ctx = NULL          // Additional user data for context
};

const httpd_uri_t svg_uri = {
    .uri = "/assets/svelte.svg", // Match your 404 error path exactly
    .method = HTTP_GET,
    .handler = svg_handler,
    .user_ctx = NULL};

httpd_handle_t start_webserver()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Server started successfully, registering URI handlers...");
        httpd_register_uri_handler(server, &hello_world_uri);
        httpd_register_uri_handler(server, &svg_uri);
        return server;
    }

    ESP_LOGE(TAG, "Failed to start server");

    return NULL;
}

extern "C" void app_main(void)
{

    // Configure
    WIFI::Config config;
    config.ssid = "ESP32-AP";
    config.password = "mypassword";
    config.channel = 6;
    config.max_connections = 4;

    WIFI::getInstance().init(config);
    WIFI::getInstance().start();

    httpd_handle_t server = start_webserver();
}