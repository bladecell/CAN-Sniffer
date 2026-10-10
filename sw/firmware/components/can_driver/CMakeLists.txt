idf_component_register(
    SRCS "can_driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_twai esp_driver_gpio freertos
    PRIV_REQUIRES hal log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
