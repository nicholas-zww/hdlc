set(OSAL_LIBOSAL_DIR "${OSAL_ROOT}/libosal")
set(OSAL_BACKEND_DIR "${OSAL_LIBOSAL_DIR}/osal/freertos")

file(GLOB OSAL_SOURCES "${OSAL_BACKEND_DIR}/*.c")

if(NOT OSAL_ENABLE_SOCKETS)
    list(FILTER OSAL_SOURCES EXCLUDE REGEX "/osal_socket_.*\\.c$")
endif()

idf_component_register(
    SRCS ${OSAL_SOURCES}
    INCLUDE_DIRS "${OSAL_LIBOSAL_DIR}/include"
    REQUIRES freertos
)

target_compile_features(${COMPONENT_LIB} PUBLIC c_std_99)
target_compile_definitions(${COMPONENT_LIB} PUBLIC OSAL_ENABLE_SOCKETS=0)
