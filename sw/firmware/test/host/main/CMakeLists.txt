idf_component_register(
    SRCS "test_main.c" "test_tinyexpr.c" "test_pid_def.cpp" "test_obd2_frame.cpp" "test_obd2_diag_collector.cpp" "test_obd2_model.cpp" "test_pid_json.cpp" "test_pid_json_numbers.cpp" "test_input_limits.cpp" "test_can_config_limits.cpp" "test_op_gate.cpp" "test_hook_cell.cpp" "test_settings_validation.cpp" "test_delete_tree.cpp" "test_body_reader.cpp" "test_error_policy.cpp" "test_atomic_file.cpp" "test_battery_calc.cpp" "test_reset_reason.cpp" "test_http_parse.cpp" "test_api_routes.cpp" "test_http_error.cpp" "test_tree_limits.cpp" "test_button_logic.cpp" "test_host_guard.cpp" "test_auth_core.cpp" "test_settings_patch.cpp"
    PRIV_REQUIRES unity tinyexpr obd2_core
)
target_sources(${COMPONENT_LIB} PRIVATE "test_wifi_scan.cpp")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_link_libraries(${COMPONENT_LIB} PUBLIC m)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../../components/sd_card/include")
