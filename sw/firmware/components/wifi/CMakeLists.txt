idf_component_register(
    SRCS "wifi.cpp" "wifi_scan.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event freertos
    PRIV_REQUIRES nvs_flash espressif__mdns esp_hw_support esp_timer log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
