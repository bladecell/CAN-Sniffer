file(GLOB SRC_FILES "*.cpp" "src/*.cpp")

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES driver
    REQUIRES can_driver wifi
    PRIV_REQUIRES esp_http_server
)