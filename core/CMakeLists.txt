
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

set(ATLAS_DIR atlas)
list(APPEND LIST_INCLUDE_DIRS ${ATLAS_DIR})

set(BACKEND_DIR backEnd)
list(APPEND LIST_INCLUDE_DIRS ${BACKEND_DIR})

set(FRONTEND_DIR frontEnd)
list(APPEND LIST_INCLUDE_DIRS ${FRONTEND_DIR})

set(OPERATORS_DIR operators)
list(APPEND LIST_INCLUDE_DIRS ${OPERATORS_DIR})

set(TRAJECTORY_DIR trajectory)
list(APPEND LIST_INCLUDE_DIRS ${TRAJECTORY_DIR})

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
set(WORLD_OBJ_DIR ${DATA_TYPES_DIR}/worldObject)
list(APPEND LIST_INCLUDE_DIRS ${WORLD_OBJ_DIR})
set(OBSERVATIONS_DIR ${DATA_TYPES_DIR}/observation)
list(APPEND LIST_INCLUDE_DIRS ${OBSERVATIONS_DIR})
set(SENSORS_DIR ${DATA_TYPES_DIR}/sensor)
list(APPEND LIST_INCLUDE_DIRS ${SENSORS_DIR})
set(SENSOR_DATA_DIR ${SENSORS_DIR}/data)
list(APPEND LIST_INCLUDE_DIRS ${SENSOR_DATA_DIR})
set(SENSOR_CAM_DIR ${SENSORS_DIR}/camera)
list(APPEND LIST_INCLUDE_DIRS ${SENSOR_CAM_DIR})
set(FRAME_DIR ${DATA_TYPES_DIR}/frame)
list(APPEND LIST_INCLUDE_DIRS ${FRAME_DIR})

set(DATA_LOADERS_DIR ${UTILS_DIR}/dataProviders)
list(APPEND LIST_INCLUDE_DIRS ${DATA_LOADERS_DIR})
#set(DATA_STORE_DIR ${DATA_LOADERS_DIR}/dataStore)
#list(APPEND LIST_INCLUDE_DIRS ${DATA_STORE_DIR})

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
        ${DATA_LOADERS_DIR}/TabularTextDS.hpp
        ${SENSOR_CAM_DIR}/Calibration.hpp
        ${SENSOR_CAM_DIR}/Camera.hpp
        ${SENSOR_DATA_DIR}/SensorData.hpp
        ${SENSOR_DATA_DIR}/Image.hpp
        ${SENSOR_DATA_DIR}/Pose.hpp
        ${SENSORS_DIR}/Sensor.hpp
        ${ATLAS_DIR}/Atlas.hpp
        ${ATLAS_DIR}/Map.hpp
        ${BACKEND_DIR}/BackEnd.hpp
        ${BACKEND_DIR}/Problem.hpp
        ${BACKEND_DIR}/BE_CalibCamCv.hpp
        ${OBSERVATIONS_DIR}/Observation.hpp
        ${OBSERVATIONS_DIR}/Point2D.hpp
        ${WORLD_OBJ_DIR}/WorldObject.hpp
        ${WORLD_OBJ_DIR}/Point3D.hpp
        ${FRONTEND_DIR}/FrontEnd.hpp
        ${FRONTEND_DIR}/FE_CalibCamCv.hpp
        ${OPERATORS_DIR}/Operator.hpp
        ${OPERATORS_DIR}/OP_ChBoardDetCv.hpp
        ${TRAJECTORY_DIR}/Trajectory.hpp
        ${TRAJECTORY_DIR}/TrajManager.hpp
        ${FRAME_DIR}/Frame.hpp
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
        ${DATA_LOADERS_DIR}/TabularTextDS.cpp
        ${SENSOR_CAM_DIR}/Calibration.cpp
        ${SENSOR_CAM_DIR}/Camera.cpp
        ${SENSORS_DIR}/Sensor.cpp
        ${SENSOR_DATA_DIR}/SensorData.cpp
        ${SENSOR_DATA_DIR}/Image.cpp
        ${SENSOR_DATA_DIR}/Pose.cpp
        ${ATLAS_DIR}/Atlas.cpp
        ${ATLAS_DIR}/Map.cpp
        ${BACKEND_DIR}/BackEnd.cpp
        ${BACKEND_DIR}/Problem.cpp
        ${BACKEND_DIR}/BE_CalibCamCv.cpp
        ${OBSERVATIONS_DIR}/Observation.cpp
        ${OBSERVATIONS_DIR}/Point2D.cpp
        ${WORLD_OBJ_DIR}/WorldObject.cpp
        ${WORLD_OBJ_DIR}/Point3D.cpp
        ${FRONTEND_DIR}/FrontEnd.cpp
        ${FRONTEND_DIR}/FE_CalibCamCv.cpp
        ${OPERATORS_DIR}/Operator.cpp
        ${OPERATORS_DIR}/OP_ChBoardDetCv.cpp
        ${TRAJECTORY_DIR}/Trajectory.cpp
        ${TRAJECTORY_DIR}/TrajManager.cpp
        ${FRAME_DIR}/Frame.cpp
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
