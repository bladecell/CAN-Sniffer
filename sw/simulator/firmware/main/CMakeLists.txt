idf_component_register(SRCS sim_can.cpp sim_engine.cpp sim_obd.cpp sim_link.cpp sim_commands.cpp
  PRIV_REQUIRES esp_driver_twai esp_driver_gpio esp_driver_usb_serial_jtag esp_timer nvs_flash
  INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror
  -Werror=extra -Werror=unused-function -Werror=unused-variable
  -Werror=unused-but-set-variable -Werror=deprecated-declarations)
