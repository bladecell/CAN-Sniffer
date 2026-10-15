idf_component_register(
    SRCS "test_main.c" "test_tinyexpr.c" "test_pid_def.cpp" "test_obd2_frame.cpp" "test_obd2_diag_collector.cpp" "test_obd2_model.cpp" "test_pid_json.cpp" "test_pid_json_numbers.cpp" "test_input_limits.cpp" "test_confirm_token.cpp"
    PRIV_REQUIRES unity tinyexpr obd2_core
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_link_libraries(${COMPONENT_LIB} PUBLIC m)
