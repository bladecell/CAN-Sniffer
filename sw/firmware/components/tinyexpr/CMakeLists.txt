idf_component_register(
    SRCS "tinyexpr.c"
    INCLUDE_DIRS "include"
    REQUIRES
)

# Optional: If you want to suppress warnings from this library during build
set_source_files_properties(tinyexpr.c PROPERTIES COMPILE_FLAGS -w)