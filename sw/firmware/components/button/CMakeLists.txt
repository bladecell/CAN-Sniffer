idf_component_register(
    SRCS "button.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio freertos
    PRIV_REQUIRES esp_timer log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
