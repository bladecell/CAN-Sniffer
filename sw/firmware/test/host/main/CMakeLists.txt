idf_component_register(
    SRCS "test_main.c" "test_tinyexpr.c"
    PRIV_REQUIRES unity tinyexpr
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
target_link_libraries(${COMPONENT_LIB} PUBLIC m)
