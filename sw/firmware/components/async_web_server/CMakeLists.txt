idf_component_register(
    SRCS "async_web_server.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server esp_timer
)