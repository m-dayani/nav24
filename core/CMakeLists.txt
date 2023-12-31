
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
set(UTILS_YAML_PARSER_DIR ${UTILS_DIR}/yamlParser)
list(APPEND LIST_INCLUDE_DIRS ${UTILS_YAML_PARSER_DIR})
set(UTILS_DT_CONV_DIR ${UTILS_DIR}/dataConversion)
list(APPEND LIST_INCLUDE_DIRS ${UTILS_DT_CONV_DIR})

set(PARAMS_DIR ${UTILS_DIR}/parameters)
list(APPEND LIST_INCLUDE_DIRS ${PARAMS_DIR})
set(PARAMS_TYPES_DIR ${PARAMS_DIR}/types)
list(APPEND LIST_INCLUDE_DIRS ${PARAMS_TYPES_DIR})

set(MESSAGING_DIR ${UTILS_DIR}/messaging)
list(APPEND LIST_INCLUDE_DIRS ${MESSAGING_DIR})

set(DATA_TYPES_DIR dataTypes)
list(APPEND LIST_INCLUDE_DIRS ${DATA_TYPES_DIR})
set(SENSOR_TYPES_DIR ${DATA_TYPES_DIR}/sensors)
list(APPEND LIST_INCLUDE_DIRS ${SENSOR_TYPES_DIR})

set(DATA_LOADERS_DIR ${UTILS_DIR}/dataProviders)
list(APPEND LIST_INCLUDE_DIRS ${DATA_LOADERS_DIR})
set(DATA_STORE_DIR ${DATA_LOADERS_DIR}/dataStore)
list(APPEND LIST_INCLUDE_DIRS ${DATA_STORE_DIR})

set(VISUALIZATION_DIR ${UTILS_DIR}/visualization)
list(APPEND LIST_INCLUDE_DIRS ${VISUALIZATION_DIR})

#set(FRONTEND_DIR frontEnd)
#set(FE_CAMERA_DIR ${FRONTEND_DIR}/Camera)
#set(FE_CAMERA_MODELS_DIR ${FE_CAMERA_DIR}/Models)
#list(APPEND LIST_INCLUDE_DIRS ${FE_CAMERA_MODELS_DIR})
#set(FE_CAMERA_UTILS_DIR ${FE_CAMERA_DIR}/Utils)
#list(APPEND LIST_INCLUDE_DIRS ${FE_CAMERA_UTILS_DIR})

set(ALL_H_FILES
        ${SYSTEM_DIR}/System.hpp
        ${DATA_LOADERS_DIR}/TabularTextWriter.hpp
        ${DATA_LOADERS_DIR}/SharedQueue.hpp
        ${UTILS_YAML_PARSER_DIR}/YamlParserCV.hpp
        ${PARAMS_TYPES_DIR}/SensorConfig.hpp
        ${PARAMS_TYPES_DIR}/Parameter.hpp
        ${PARAMS_TYPES_DIR}/CamParams.hpp
        ${PARAMS_TYPES_DIR}/IMU_Params.hpp
        ${PARAMS_TYPES_DIR}/DS_Params.hpp
        ${PARAMS_DIR}/ParameterServer.hpp
        ${MESSAGING_DIR}/Message.hpp
        ${MESSAGING_DIR}/Channel.hpp
        ${DATA_LOADERS_DIR}/BaseLoader.hpp
        ${DATA_LOADERS_DIR}/EurocLoader.hpp
        ${VISUALIZATION_DIR}/Visualization.hpp
        ${SENSOR_TYPES_DIR}/Image.hpp
        ${SENSOR_TYPES_DIR}/IMU.hpp
        ${SENSOR_TYPES_DIR}/ImageFilter.hpp
        ${SENSOR_TYPES_DIR}/ImageHook.hpp
        ${SENSOR_TYPES_DIR}/IMU_Hook.hpp
        ${UTILS_DT_CONV_DIR}/DataConversion.hpp
        ${DATA_STORE_DIR}/PoseDS.hpp
        ${DATA_STORE_DIR}/ImageDS.hpp
        ${DATA_STORE_DIR}/IMU_DS.hpp
        ${DATA_STORE_DIR}/TabularTextDS.hpp
)
set(ALL_SRC_FILES
        ${SYSTEM_DIR}/System.cpp
        ${DATA_LOADERS_DIR}/TabularTextWriter.cpp
        ${DATA_LOADERS_DIR}/SharedQueue.cpp
        ${UTILS_YAML_PARSER_DIR}/YamlParserCV.cpp
        ${PARAMS_TYPES_DIR}/SensorConfig.cpp
        ${PARAMS_TYPES_DIR}/Parameter.cpp
        ${PARAMS_TYPES_DIR}/CamParams.cpp
        ${PARAMS_TYPES_DIR}/IMU_Params.cpp
        ${PARAMS_TYPES_DIR}/DS_Params.cpp
        ${PARAMS_DIR}/ParameterServer.cpp
        ${MESSAGING_DIR}/Message.cpp
        ${MESSAGING_DIR}/Channel.cpp
        ${DATA_LOADERS_DIR}/BaseLoader.cpp
        ${DATA_LOADERS_DIR}/EurocLoader.cpp
        ${VISUALIZATION_DIR}/Visualization.cpp
        ${SENSOR_TYPES_DIR}/Image.cpp
        ${SENSOR_TYPES_DIR}/IMU.cpp
        ${SENSOR_TYPES_DIR}/ImageFilter.cpp
        ${SENSOR_TYPES_DIR}/ImageHook.cpp
        ${SENSOR_TYPES_DIR}/IMU_Hook.cpp
        ${UTILS_DT_CONV_DIR}/DataConversion.cpp
        ${DATA_STORE_DIR}/PoseDS.cpp
        ${DATA_STORE_DIR}/ImageDS.cpp
        ${DATA_STORE_DIR}/IMU_DS.cpp
        ${DATA_STORE_DIR}/TabularTextDS.cpp
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
