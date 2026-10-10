cmake_minimum_required(VERSION 3.16)

# Host (linux target) unit tests. Only hardware-free components are pulled in.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components/tinyexpr")
set(COMPONENTS main tinyexpr unity)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(host_tests)
