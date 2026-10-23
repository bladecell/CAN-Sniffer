cmake_minimum_required(VERSION 3.16)
get_filename_component(FIRMWARE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
string(FIND "${CMAKE_BINARY_DIR}/" "${FIRMWARE_DIR}/" inside_firmware)
if(inside_firmware EQUAL 0)
  message(FATAL_ERROR "Use an external build directory, e.g. -B /tmp/opencode/p28/consolidation/build")
endif()
if(NOT "$ENV{IDF_COMPONENT_MANAGER}" STREQUAL "0")
  message(FATAL_ERROR "Set IDF_COMPONENT_MANAGER=0; existing components only, no resolution/network")
endif()
set(EXTRA_COMPONENT_DIRS
  "${FIRMWARE_DIR}/components/obd2"
  "${FIRMWARE_DIR}/components/obd2_simulator"
  "${FIRMWARE_DIR}/components/can_driver"
  "${FIRMWARE_DIR}/components/tinyexpr"
  "${FIRMWARE_DIR}/managed_components/espressif__cjson")
set(COMPONENTS main)
set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig" CACHE FILEPATH "Isolated canonical harness config" FORCE)
if(NOT EXISTS "${SDKCONFIG}")
  file(READ "${FIRMWARE_DIR}/sdkconfig" harness_config)
  if(NOT harness_config MATCHES "CONFIG_SNIFFER_HW_REV_V1_1=y" OR
     NOT harness_config MATCHES "CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y")
    message(FATAL_ERROR "Only approved V1_1/16MB hardware is supported")
  endif()
  string(REPLACE "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"partitions_v1.csv\""
    "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"${FIRMWARE_DIR}/partitions_v1.csv\""
    harness_config "${harness_config}")
  file(WRITE "${SDKCONFIG}" "${harness_config}")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(obd_can_lifecycle_tests)
