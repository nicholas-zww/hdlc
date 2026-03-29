set(OSAL_LIBFIPC_DIR "${OSAL_ROOT}/libfipc")

file(GLOB FIPC_SOURCES "${OSAL_LIBFIPC_DIR}/*.c")
list(FILTER FIPC_SOURCES EXCLUDE REGEX "/tests/")
list(FILTER FIPC_SOURCES EXCLUDE REGEX "/examples/")

idf_component_register(
    SRCS ${FIPC_SOURCES}
    INCLUDE_DIRS
        "${OSAL_LIBFIPC_DIR}"
        "${CMAKE_SOURCE_DIR}/include"
    REQUIRES
        libosal
)

target_compile_features(${COMPONENT_LIB} PUBLIC c_std_99)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
target_compile_definitions(${COMPONENT_LIB} PUBLIC FIPC_FRAME_PROTOCOL_BUFFER_SIZE=256)
