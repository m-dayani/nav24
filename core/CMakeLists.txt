
project(nav_core)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(OpenCV 4.2.0 REQUIRED)
MESSAGE("-- OpenCV Version: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

# Note: One can add g2o as an external package:
#find_package(g2o REQUIRED)

#list(APPEND LIST_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})

set(SYSTEM_DIR system)
list(APPEND LIST_INCLUDE_DIRS ${SYSTEM_DIR})

set(UTILS_DIR tools)
list(APPEND LIST_INCLUDE_DIRS ${UTILS_DIR})
set(UTILS_DT_CONV_DIR ${UTILS_DIR}/dataConversion)
list(APPEND LIST_INCLUDE_DIRS ${UTILS_DT_CONV_DIR})

set(PARAMS_DIR ${UTILS_DIR}/parameters)
list(APPEND LIST_INCLUDE_DIRS ${PARAMS_DIR})

set(MESSAGING_DIR ${UTILS_DIR}/messaging)
list(APPEND LIST_INCLUDE_DIRS ${MESSAGING_DIR})

set(DATA_TYPES_DIR dataTypes)
list(APPEND LIST_INCLUDE_DIRS ${DATA_TYPES_DIR})
set(SENSORS_DIR ${DATA_TYPES_DIR}/sensor)
list(APPEND LIST_INCLUDE_DIRS ${SENSORS_DIR})
set(SENSOR_DATA_DIR ${SENSORS_DIR}/data)
list(APPEND LIST_INCLUDE_DIRS ${SENSOR_DATA_DIR})
set(SENSOR_CAM_DIR ${SENSORS_DIR}/camera)
list(APPEND LIST_INCLUDE_DIRS ${SENSOR_CAM_DIR})

set(DATA_LOADERS_DIR ${UTILS_DIR}/dataProviders)
list(APPEND LIST_INCLUDE_DIRS ${DATA_LOADERS_DIR})
set(DATA_STORE_DIR ${DATA_LOADERS_DIR}/dataStore)
list(APPEND LIST_INCLUDE_DIRS ${DATA_STORE_DIR})

set(VISUALIZATION_DIR ${UTILS_DIR}/visualization)
list(APPEND LIST_INCLUDE_DIRS ${VISUALIZATION_DIR})


set(ALL_H_FILES
        ${SYSTEM_DIR}/System.hpp
        ${UTILS_DT_CONV_DIR}/YamlParserCV.hpp
        ${UTILS_DT_CONV_DIR}/DataConversion.hpp
        ${PARAMS_DIR}/Parameter.hpp
        ${PARAMS_DIR}/ParameterServer.hpp
        ${PARAMS_DIR}/ParameterBlueprint.h
        ${MESSAGING_DIR}/Message.hpp
        ${MESSAGING_DIR}/Channel.hpp
        ${DATA_LOADERS_DIR}/DataStore.hpp
        ${DATA_STORE_DIR}/TabularTextDS.hpp
        ${SENSOR_CAM_DIR}/Calibration.hpp
        ${SENSOR_CAM_DIR}/Camera.hpp
        ${SENSOR_DATA_DIR}/SensorData.hpp
        ${SENSORS_DIR}/Sensor.hpp
)
set(ALL_SRC_FILES
        ${SYSTEM_DIR}/System.cpp
        ${UTILS_DT_CONV_DIR}/YamlParserCV.cpp
        ${UTILS_DT_CONV_DIR}/DataConversion.cpp
        ${PARAMS_DIR}/Parameter.cpp
        ${PARAMS_DIR}/ParameterServer.cpp
        ${MESSAGING_DIR}/Message.cpp
        ${MESSAGING_DIR}/Channel.cpp
        ${DATA_LOADERS_DIR}/DataStore.cpp
        ${DATA_STORE_DIR}/TabularTextDS.cpp
        ${SENSOR_CAM_DIR}/Calibration.cpp
        ${SENSOR_CAM_DIR}/Camera.cpp
        ${SENSORS_DIR}/Sensor.cpp
        ${SENSOR_DATA_DIR}/SensorData.cpp

)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${LIST_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
#        ${Pangolin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
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

target_link_libraries(${PROJECT_NAME}
        ${EIGEN3_LIBS}
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
#        ${Pangolin_LIBRARIES}
#        ${CMAKE_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so
#        ${CMAKE_SOURCE_DIR}/thirdparty/g2o/lib/libg2o.so
        -lboost_system
        -lboost_filesystem
        -lboost_serialization
#        -lcrypto
)

install(TARGETS ${PROJECT_NAME}
        EXPORT NAV_CORE
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${ALL_H_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
