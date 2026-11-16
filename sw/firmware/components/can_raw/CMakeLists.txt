idf_component_register(
    SRCS "can_raw.cpp"
    INCLUDE_DIRS "include"
    REQUIRES can_driver freertos
    PRIV_REQUIRES heap log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
