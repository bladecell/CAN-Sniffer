#include <stdlib.h>
#include "unity.h"

void run_tinyexpr_tests(void);
void run_pid_def_tests(void);
void run_obd2_frame_tests(void);
void run_obd2_diag_collector_tests(void);
void run_obd2_model_tests(void);
void run_pid_json_tests(void);
void run_pid_json_numbers_tests(void);
void run_can_config_limits_tests(void);
void run_op_gate_tests(void);
void run_hook_cell_tests(void);
void run_settings_validation_tests(void);
void run_delete_tree_tests(void);
void run_body_reader_tests(void);
void run_http_body_policy_tests(void);
void run_error_policy_tests(void);
void run_supervisor_core_tests(void);
void run_power_settings_tests(void);
void run_manual_sleep_http_tests(void);
void run_atomic_file_tests(void);
void run_battery_calc_tests(void);
void run_reset_reason_tests(void);
void run_http_parse_tests(void);
void run_api_routes_tests(void);
void run_http_error_tests(void);
void run_tree_limits_tests(void);
void run_button_logic_tests(void);
void run_host_guard_tests(void);
void run_auth_core_tests(void);
void run_wifi_scan_tests(void);
void run_settings_patch_tests(void);
void run_obd2_patch_tests(void);
void run_json_budget_tests(void);
void run_settings_merge_tests(void);
void run_input_limits_tests(void);
void run_single_flight_tests(void);
void run_response_rules_tests(void);

/* The linux IDF port starts FreeRTOS and calls app_main(); exit with the failure count. */
void app_main(void)
{
    UNITY_BEGIN();
    run_tinyexpr_tests();
    run_pid_def_tests();
    run_obd2_frame_tests();
    run_obd2_diag_collector_tests();
    run_obd2_model_tests();
    run_pid_json_tests();
    run_pid_json_numbers_tests();
    run_can_config_limits_tests();
    run_op_gate_tests();
    run_hook_cell_tests();
    run_settings_validation_tests();
    run_delete_tree_tests();
    run_body_reader_tests();
    run_http_body_policy_tests();
    run_error_policy_tests();
    run_supervisor_core_tests();
    run_power_settings_tests();
    run_manual_sleep_http_tests();
    run_atomic_file_tests();
    run_battery_calc_tests();
    run_reset_reason_tests();
    run_http_parse_tests();
    run_api_routes_tests();
    run_http_error_tests();
    run_tree_limits_tests();
    run_button_logic_tests();
    run_host_guard_tests();
    run_auth_core_tests();
    run_wifi_scan_tests();
    run_settings_patch_tests();
    run_obd2_patch_tests();
    run_json_budget_tests();
    run_settings_merge_tests();
    run_input_limits_tests();
    run_single_flight_tests();
    run_response_rules_tests();
    exit(UNITY_END() == 0 ? 0 : 1);
}
