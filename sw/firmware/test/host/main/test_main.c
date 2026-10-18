#include <stdlib.h>
#include "unity.h"

void run_tinyexpr_tests(void);
void run_pid_def_tests(void);
void run_obd2_frame_tests(void);
void run_obd2_diag_collector_tests(void);
void run_obd2_model_tests(void);
void run_pid_json_tests(void);
void run_pid_json_numbers_tests(void);
void run_confirm_token_tests(void);
void run_can_config_limits_tests(void);
void run_op_gate_tests(void);
void run_hook_cell_tests(void);
void run_settings_validation_tests(void);
void run_delete_tree_tests(void);
void run_body_reader_tests(void);
void run_error_policy_tests(void);
void run_atomic_file_tests(void);

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
    run_confirm_token_tests();
    run_can_config_limits_tests();
    run_op_gate_tests();
    run_hook_cell_tests();
    run_settings_validation_tests();
    run_delete_tree_tests();
    run_body_reader_tests();
    run_error_policy_tests();
    run_atomic_file_tests();
    exit(UNITY_END() == 0 ? 0 : 1);
}
