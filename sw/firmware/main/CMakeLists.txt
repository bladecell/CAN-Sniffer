file(GLOB SRC_FILES "*.cpp" "src/*.cpp")

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES driver can_driver wifi async_web_server esp_http_server tinyexpr esp_adc sd_card
    PRIV_REQUIRES json 
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

littlefs_create_partition_image(www "../../frontend/dist" FLASH_IN_PROJECT)