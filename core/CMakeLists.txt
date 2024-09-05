
project(nav_core)

#set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Required Packages

find_package(Eigen3 3.1.0 REQUIRED)

find_package(OpenCV 4.2.0 REQUIRED)

find_package(Boost COMPONENTS system filesystem REQUIRED)

find_package(glog REQUIRED)

# Optional Packages

#find_package(Ceres)
find_package(g2o)
find_package(DBoW2)

if (USE_PANGOLIN)
    find_package(Pangolin)
endif (USE_PANGOLIN)
if (Pangolin_FOUND)
    add_compile_definitions(LIB_PANGOLIN_FOUND=1)
#    message("-- ${Pangolin_LIBRARIES}")
endif (Pangolin_FOUND)

if (USE_SERIAL)
    find_package(serial)
endif (USE_SERIAL)
if (serial_FOUND)
    add_compile_definitions(LIB_SERIAL_FOUND=1)
endif (serial_FOUND)

# Python Library
if (USE_PYTHON)
    find_package(Python COMPONENTS Interpreter Development)
endif (USE_PYTHON)
if (Python_FOUND)
    add_compile_definitions(LIB_PYTHON_FOUND=1)
endif (Python_FOUND)

if (USE_ONNX_RUNTIME)
    find_package(onnxruntime)
endif (USE_ONNX_RUNTIME)
if (onnxruntime_FOUND)
    add_compile_definitions(LIB_ONNX_RUNTIME_FOUND=1)
endif (onnxruntime_FOUND)


include(ProjectFiles.cmake)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${LIST_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${serial_INCLUDE_DIRS}
        ${Python_INCLUDE_DIRS}
        ${onnxruntime_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${DBoW2_INCLUDE_DIRS}
#        ${CERES_INCLUDE_DIRS}
)

list(APPEND LINKLIBS ${Python_LIBRARIES})

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
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${serial_LIBRARIES}
        ${Python_LIBRARIES}
        ${onnxruntime_LIBRARIES}
        glog::glog
        -lboost_system
        -lboost_filesystem
        ${CMAKE_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so
        ${CMAKE_SOURCE_DIR}/thirdparty/g2o/lib/libg2o.so
#        ${EIGEN3_LIBS}
#        -lboost_serialization
#        -lcrypto
#        ${CERES_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
        EXPORT NAV_CORE
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${ALL_H_FILES} DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR})



