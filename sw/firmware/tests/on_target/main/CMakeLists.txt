idf_component_register(SRCS "lifecycle_tests.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES obd2 can_driver tinyexpr unity)
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=xTaskCreatePinnedToCore")
