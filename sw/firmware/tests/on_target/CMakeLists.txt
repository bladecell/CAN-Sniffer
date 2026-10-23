cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/obd2"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/obd2_simulator"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/can_driver"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tinyexpr"
    "${CMAKE_CURRENT_LIST_DIR}/../../managed_components/espressif__cjson")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(obd_can_lifecycle_tests)
