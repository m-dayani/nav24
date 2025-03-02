
project(nav_core)

#set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Required Packages

find_package(Eigen3 3.1.0 REQUIRED)

find_package(OpenCV 4.5.1 REQUIRED)

find_package(Boost COMPONENTS system filesystem REQUIRED)

find_package(glog REQUIRED)

# Optional Packages

# Optimization
#find_package(Ceres)
find_package(g2o CONFIG)
#message("-- g2o libs: ${g2o_LIBRARIES}")

# VPR
find_package(DBoW2 CONFIG)
if (DBoW2_FOUND)
    add_compile_definitions(LIB_DBOW2_FOUND=1)
    #message("-- DBoW2 libs: ${DBoW2_LIBRARIES}")
endif (DBoW2_FOUND)

# Pangolin
if (USE_PANGOLIN)
    find_package(Pangolin)
endif (USE_PANGOLIN)
if (Pangolin_FOUND)
    add_compile_definitions(LIB_PANGOLIN_FOUND=1)
#    message("-- ${Pangolin_LIBRARIES}")
endif (Pangolin_FOUND)

# Serial
if (USE_SERIAL)
    find_package(serial)
endif (USE_SERIAL)
if (serial_FOUND)
    add_compile_definitions(LIB_SERIAL_FOUND=1)
endif (serial_FOUND)

# ONNX RT
if (USE_ONNX_RUNTIME)
    find_package(onnxruntime)
endif (USE_ONNX_RUNTIME)
if (onnxruntime_FOUND)
    add_compile_definitions(LIB_ONNX_RUNTIME_FOUND=1)
    message("-- Found ONNX RT: ${onnxruntime_LIBRARIES}")
endif (onnxruntime_FOUND)

# AprilTag
find_package(apriltag)
if (apriltag_FOUND)
    add_compile_definitions(LIB_APRILTAG_FOUND=1)
    message("-- Found apriltag library")
endif (apriltag_FOUND)


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
        ${g2o_INCLUDE_DIRS}
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
        ${DBoW2_LIBRARIES}
        ${onnxruntime_LIBRARIES}
        -lonnxruntime
        apriltag::apriltag
        glog::glog
        g2o::core g2o::types_sba g2o::types_slam3d
        -lboost_system
        -lboost_filesystem
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

#install(FILES ${ALL_H_FILES} DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR})



