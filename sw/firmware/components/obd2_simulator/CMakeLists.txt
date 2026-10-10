idf_component_register(
    SRCS "obd2_simulator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos
    PRIV_REQUIRES can_driver obd2 esp_hw_support log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
