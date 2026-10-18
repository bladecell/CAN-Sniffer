idf_component_register(
    SRCS "test_main.c" "test_tinyexpr.c" "test_pid_def.cpp" "test_obd2_frame.cpp" "test_obd2_diag_collector.cpp" "test_obd2_model.cpp" "test_pid_json.cpp" "test_pid_json_numbers.cpp" "test_input_limits.cpp" "test_confirm_token.cpp" "test_can_config_limits.cpp" "test_op_gate.cpp" "test_hook_cell.cpp" "test_settings_validation.cpp" "test_delete_tree.cpp"
    PRIV_REQUIRES unity tinyexpr obd2_core
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_link_libraries(${COMPONENT_LIB} PUBLIC m)
