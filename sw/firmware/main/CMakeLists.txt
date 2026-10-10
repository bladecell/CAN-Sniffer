file(GLOB SRC_FILES "*.cpp" "src/*.cpp")

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES can_driver obd2 obd2_simulator wifi async_web_server sd_card esp_http_server freertos
    PRIV_REQUIRES esp_driver_gpio esp_adc esp_hw_support esp_pm esp_timer esp_system http_parser json joltwallet__littlefs log nvs_flash
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_custom_target(build_svelte_frontend
    COMMAND bun run build
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../frontend"
    COMMENT "Building Svelte frontend with Bun..."
)

littlefs_create_partition_image(www "../../frontend/dist" FLASH_IN_PROJECT)

add_dependencies(littlefs_www_bin build_svelte_frontend)