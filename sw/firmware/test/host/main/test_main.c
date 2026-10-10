#include <stdlib.h>
#include "unity.h"

void run_tinyexpr_tests(void);

/* The linux IDF port starts FreeRTOS and calls app_main(); exit with the failure count. */
void app_main(void)
{
    UNITY_BEGIN();
    run_tinyexpr_tests();
    exit(UNITY_END() == 0 ? 0 : 1);
}
