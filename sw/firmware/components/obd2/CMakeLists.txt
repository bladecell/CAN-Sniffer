idf_component_register(
    SRCS "obd2.cpp" "obd2_data_model.cpp" "pid_def.cpp"
    INCLUDE_DIRS "include"
    REQUIRES can_driver freertos
    PRIV_REQUIRES tinyexpr log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
