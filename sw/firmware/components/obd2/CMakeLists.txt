idf_component_register(
    SRCS "obd2.cpp" "obd2_discovery.cpp" "obd2_recovery.cpp" "obd2_diag.cpp" "obd2_data_model.cpp" "pid_def.cpp" "obd2_frame.cpp" "pid_json.cpp"
    INCLUDE_DIRS "include"
    REQUIRES can_driver freertos json
    PRIV_REQUIRES tinyexpr log
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
