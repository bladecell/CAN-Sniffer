idf_component_register(SRCS "lifecycle_tests.cpp" "../../../main/src/settings.cpp"
                       INCLUDE_DIRS "." "../../../main/include" "../../../components/wifi/include"
                       REQUIRES obd2 obd2_simulator can_driver tinyexpr unity nvs_flash esp_wifi esp_event esp_timer)
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=xTaskCreatePinnedToCore")
