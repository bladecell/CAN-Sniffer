idf_component_register(
    SRCS "can_driver.cpp" "can_driver_twai.cpp" "can_driver_io.cpp" "can_driver_state.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_twai esp_driver_gpio freertos
    PRIV_REQUIRES hal log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
