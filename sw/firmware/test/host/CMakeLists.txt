cmake_minimum_required(VERSION 3.16)

# Host (linux target) unit tests. Only hardware-free components are pulled in.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components/tinyexpr"
                         "${CMAKE_CURRENT_LIST_DIR}/components/obd2_core"
                         "${CMAKE_CURRENT_LIST_DIR}/components/can_driver"
                         "${CMAKE_CURRENT_LIST_DIR}/../../managed_components/espressif__cjson")
set(COMPONENTS main tinyexpr obd2_core can_driver espressif__cjson unity)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(host_tests)
