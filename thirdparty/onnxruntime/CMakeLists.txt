# ONNX Runtime and CUDA (for loading dl models like YOLO)
set(ONNX_RUNTIME_PATH "/opt/onnxruntime" CACHE STRING "ONNX Runtime directory path")
set(CUDA_LIBRARIES_PATH "/opt/cuda11" CACHE STRING "CUDA libraries path compatible with the ONNX Runtime")

# Create Onnxruntime imported target
add_library(onnxruntime SHARED IMPORTED)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(ONNX_IMPORTED_LOCATION "${ONNX_RUNTIME_PATH}/lib/onnxruntime.dll")
    set(ONNX_IMPORTED_IMPLIB "${ONNX_RUNTIME_PATH}/lib/onnxruntime.lib")
    set(ONNX_INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_PATH}/include")
    if(NOT EXISTS ${ONNX_IMPORTED_LOCATION})
        message(FATAL_ERROR "Excepted Onnxruntime library '${ONNX_IMPORTED_LOCATION}' does not exist. Please specify a valid ONNX_RUNTIME_PATH.(current: ${ONNX_RUNTIME_PATH})")
    endif()
    if(NOT EXISTS ${ONNX_IMPORTED_IMPLIB})
        message(FATAL_ERROR "Excepted Onnxruntime imported library '${ONNX_IMPORTED_IMPLIB}' does not exist. Please specify a valid ONNX_RUNTIME_PATH.(current: ${ONNX_RUNTIME_PATH})")
    endif()
    if(NOT EXISTS ${ONNX_INTERFACE_INCLUDE_DIRECTORIES})
        message(FATAL_ERROR "Excepted Onnxruntime include directory '${ONNX_INTERFACE_INCLUDE_DIRECTORIES}' does not exist. Please specify a valid ONNX_RUNTIME_PATH.(current: ${ONNX_RUNTIME_PATH})")
    endif()

    set_target_properties(onnxruntime PROPERTIES
            IMPORTED_LOCATION ${ONNX_IMPORTED_LOCATION}
            IMPORTED_IMPLIB   ${ONNX_IMPORTED_IMPLIB}
            INTERFACE_INCLUDE_DIRECTORIES ${ONNX_INTERFACE_INCLUDE_DIRECTORIES}
    )
elseif(UNIX)
    set(ONNX_IMPORTED_LOCATION "${ONNX_RUNTIME_PATH}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}onnxruntime${CMAKE_SHARED_LIBRARY_SUFFIX}")
    set(ONNX_INTERFACE_INCLUDE_DIRECTORIES "${ONNX_RUNTIME_PATH}/include")
    if(NOT EXISTS ${ONNX_IMPORTED_LOCATION})
        message(FATAL_ERROR "Excepted Onnxruntime library '${ONNX_IMPORTED_LOCATION}' does not exist. Please specify a valid ONNX_RUNTIME_PATH.(current: ${ONNX_RUNTIME_PATH})")
    endif()
    if(NOT EXISTS ${ONNX_INTERFACE_INCLUDE_DIRECTORIES})
        message(FATAL_ERROR "Excepted Onnxruntime include directory '${ONNX_INTERFACE_INCLUDE_DIRECTORIES}' does not exist. Please specify a valid ONNX_RUNTIME_PATH.(current: ${ONNX_RUNTIME_PATH})")
    endif()

    set_target_properties(onnxruntime PROPERTIES
            IMPORTED_LOCATION ${ONNX_IMPORTED_LOCATION}
            INTERFACE_INCLUDE_DIRECTORIES ${ONNX_INTERFACE_INCLUDE_DIRECTORIES}
    )
else()
    message(FATAL_ERROR "Unsupported target platform.")
endif()