# Keep this component limited to the project's own sources.
# Using CMAKE_SOURCE_DIR recursively pulls in PlatformIO dependency examples
# from .pio/libdeps, which creates duplicate setup/loop/app_main definitions.
file(GLOB_RECURSE app_sources
    "${COMPONENT_DIR}/*.cpp"
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "../include"
    REQUIRES libfipc libosal
)
