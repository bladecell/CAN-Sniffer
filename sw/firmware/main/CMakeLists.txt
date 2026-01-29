file(GLOB SRC_FILES "*.cpp" "src/*.cpp")

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES driver can_driver wifi async_web_server esp_http_server tinyexpr
    PRIV_REQUIRES json 
)