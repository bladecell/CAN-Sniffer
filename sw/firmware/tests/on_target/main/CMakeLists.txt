idf_component_register(
  SRCS "app_main.cpp" "lifecycle_tests.cpp" "p28_lifecycle.cpp" "task_injection.cpp" "nvs_fixture.cpp"
       "${FIRMWARE_DIR}/main/src/settings.cpp"
  INCLUDE_DIRS "." "${FIRMWARE_DIR}/main/include" "${FIRMWARE_DIR}/components/wifi/include"
  REQUIRES obd2 obd2_simulator can_driver tinyexpr unity nvs_flash esp_wifi esp_event esp_timer esp_hw_support)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=xTaskCreatePinnedToCore")
