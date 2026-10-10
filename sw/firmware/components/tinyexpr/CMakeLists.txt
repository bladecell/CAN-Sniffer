idf_component_register(
    SRCS "tinyexpr.c"
    INCLUDE_DIRS "include"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
# Vendored library: new_expr() deliberately allocates one pointer less than sizeof(te_expr)
# for leaf nodes, which trips GCC's -Warray-bounds false positive.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-array-bounds)
