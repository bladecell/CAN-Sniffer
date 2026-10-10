idf_component_register(
    SRCS "sd_card.cpp"
    INCLUDE_DIRS "include"
    REQUIRES json esp_driver_gpio esp_driver_sdmmc fatfs sdmmc soc freertos
    PRIV_REQUIRES log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
