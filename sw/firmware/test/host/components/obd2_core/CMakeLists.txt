# Host build of the hardware-free OBD2 sources, compiled straight from components/obd2 (no copies).
set(OBD2 "${CMAKE_CURRENT_LIST_DIR}/../../../../components/obd2")

idf_component_register(
    SRCS "${OBD2}/pid_def.cpp" "${OBD2}/obd2_data_model.cpp" "${OBD2}/obd2_frame.cpp" "${OBD2}/pid_json.cpp"
    INCLUDE_DIRS "${OBD2}/include"
    REQUIRES freertos esp_common log tinyexpr can_driver espressif__cjson
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
