
project(nav_core)

#set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Required Packages

find_package(Eigen3 3.1.0 REQUIRED)

find_package(OpenCV 4.2.0 REQUIRED)
MESSAGE("-- OpenCV Version: ${OpenCV_VERSION}")

find_package(glog REQUIRED)

# Optional Packages

if (USE_SERIAL)
    find_package(serial)
endif ()
if (serial_FOUND)
    add_compile_definitions(LIB_SERIAL_FOUND=1)
endif ()

#find_package(Ceres)

#find_package(g2o)

# Python Library
if (USE_PYTHON)
    find_package(Python COMPONENTS Interpreter Development)
endif ()
if (Python_FOUND)
    add_compile_definitions(LIB_PYTHON_FOUND=1)
endif ()

if (USE_ONNX_RUNTIME)
    find_package(onnxruntime)
endif ()
if (onnxruntime_FOUND)
    add_compile_definitions(LIB_ONNX_RUNTIME_FOUND=1)
endif ()


include(ProjectFiles.cmake)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${LIST_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
#        ${Pangolin_INCLUDE_DIRS}
#        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${serial_INCLUDE_DIRS}
        ${Python_INCLUDE_DIRS}
        ${onnxruntime_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        ${ALL_H_FILES}
        ${ALL_SRC_FILES}
)

list(APPEND ALL_BUILD_INTERFACE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
foreach(INCLUDE_DIR IN LISTS LIST_INCLUDE_DIRS)
    list(APPEND ALL_BUILD_INTERFACE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${INCLUDE_DIR}>)
endforeach()
#message(${ALL_BUILD_INTERFACE_DIRS})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${ALL_BUILD_INTERFACE_DIRS}
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_directories(${PROJECT_NAME} PUBLIC
        "${CUDA_LIBRARIES_PATH}/lib64"
#        ${Python_LIBRARY_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${EIGEN3_LIBS}
        ${OpenCV_LIBS}
        ${Python_LIBRARIES}
#        ${CERES_LIBRARIES}
#        ${Pangolin_LIBRARIES}
#        ${CMAKE_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so
#        ${CMAKE_SOURCE_DIR}/thirdparty/g2o/lib/libg2o.so
        ${serial_LIBRARIES}
        -lboost_system
        -lboost_filesystem
        -lboost_serialization
#        -lcrypto
        ${onnxruntime_LIBRARIES}
        glog::glog
)

install(TARGETS ${PROJECT_NAME}
        EXPORT NAV_CORE
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${ALL_H_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
