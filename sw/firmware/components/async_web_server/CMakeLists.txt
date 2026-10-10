idf_component_register(
    SRCS "async_web_server.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server freertos
    PRIV_REQUIRES esp_timer log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
