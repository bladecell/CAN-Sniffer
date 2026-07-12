file(GLOB SRC_FILES "*.cpp" "src/*.cpp")

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES driver can_driver wifi async_web_server esp_http_server tinyexpr esp_adc sd_card
    PRIV_REQUIRES json 
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_custom_target(build_svelte_frontend
    COMMAND bun run build
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../frontend"
    COMMENT "Building Svelte frontend with Bun..."
)

littlefs_create_partition_image(www "../../frontend/dist" FLASH_IN_PROJECT)

littlefs_create_partition_image(storage "../default_storage" FLASH_IN_PROJECT)

add_dependencies(littlefs_www_bin build_svelte_frontend)