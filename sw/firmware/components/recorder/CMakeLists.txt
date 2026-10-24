idf_component_register(
    SRCS "recorder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos obd2 sd_card
    PRIV_REQUIRES espressif__cjson esp_app_format esp_timer log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
