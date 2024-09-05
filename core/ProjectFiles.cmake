
#list(APPEND LIST_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})

set(SYSTEM_DIR system)
list(APPEND LIST_INCLUDE_DIRS ${SYSTEM_DIR})

set(ATLAS_DIR scene)
list(APPEND LIST_INCLUDE_DIRS ${ATLAS_DIR})
set(WORLD_OBJ_DIR ${ATLAS_DIR}/worldObject)
list(APPEND LIST_INCLUDE_DIRS ${WORLD_OBJ_DIR})

set(BACKEND_DIR backEnd)
list(APPEND LIST_INCLUDE_DIRS ${BACKEND_DIR})

set(FRONTEND_DIR frontEnd)
list(APPEND LIST_INCLUDE_DIRS ${FRONTEND_DIR})

set(OPERATORS_DIR operators)
list(APPEND LIST_INCLUDE_DIRS ${OPERATORS_DIR})
set(OBJ_TRACK_DIR ${OPERATORS_DIR}/objTracking)
list(APPEND LIST_INCLUDE_DIRS ${OBJ_TRACK_DIR})
set(OBJ_DET_DIR ${OPERATORS_DIR}/objDetection)
list(APPEND LIST_INCLUDE_DIRS ${OBJ_DET_DIR})
set(OBJ_ASSOC_DIR ${OPERATORS_DIR}/objAssoc)
list(APPEND LIST_INCLUDE_DIRS ${OBJ_ASSOC_DIR})
set(MAP_INIT_DIR ${OPERATORS_DIR}/mapInit)
list(APPEND LIST_INCLUDE_DIRS ${MAP_INIT_DIR})

set(TRAJECTORY_DIR trajectory)
list(APPEND LIST_INCLUDE_DIRS ${TRAJECTORY_DIR})
set(POSE_DIR ${TRAJECTORY_DIR}/pose)
list(APPEND LIST_INCLUDE_DIRS ${POSE_DIR})

set(SENSOR_DATA_DIR sensorData)
list(APPEND LIST_INCLUDE_DIRS ${SENSOR_DATA_DIR})
set(OBSERVATIONS_DIR ${SENSOR_DATA_DIR}/observation)
list(APPEND LIST_INCLUDE_DIRS ${OBSERVATIONS_DIR})

set(SENSORS_DIR sensor)
list(APPEND LIST_INCLUDE_DIRS ${SENSORS_DIR})
set(SENSOR_CAM_DIR ${SENSORS_DIR}/camera)
list(APPEND LIST_INCLUDE_DIRS ${SENSOR_CAM_DIR})
set(SENSOR_CAM_MODELS_DIR ${SENSOR_CAM_DIR}/models)
list(APPEND LIST_INCLUDE_DIRS ${SENSOR_CAM_MODELS_DIR})

set(DATA_TYPES_DIR dataTypes)
list(APPEND LIST_INCLUDE_DIRS ${DATA_TYPES_DIR})
set(FRAME_DIR ${DATA_TYPES_DIR}/frame)
list(APPEND LIST_INCLUDE_DIRS ${FRAME_DIR})
set(OPTIM_DT_DIR ${DATA_TYPES_DIR}/optimization)
list(APPEND LIST_INCLUDE_DIRS ${OPTIM_DT_DIR})

set(UTILS_DIR tools)
list(APPEND LIST_INCLUDE_DIRS ${UTILS_DIR})
set(UTILS_DT_CONV_DIR ${UTILS_DIR}/dataConversion)
list(APPEND LIST_INCLUDE_DIRS ${UTILS_DT_CONV_DIR})

set(PARAMS_DIR ${UTILS_DIR}/parameters)
list(APPEND LIST_INCLUDE_DIRS ${PARAMS_DIR})

set(MESSAGING_DIR ${UTILS_DIR}/messaging)
list(APPEND LIST_INCLUDE_DIRS ${MESSAGING_DIR})

set(DATA_LOADERS_DIR ${UTILS_DIR}/dataProviders)
list(APPEND LIST_INCLUDE_DIRS ${DATA_LOADERS_DIR})
#set(DATA_STORE_DIR ${DATA_LOADERS_DIR}/dataStore)
#list(APPEND LIST_INCLUDE_DIRS ${DATA_STORE_DIR})

set(VISUALIZATION_DIR ${UTILS_DIR}/visualization)
list(APPEND LIST_INCLUDE_DIRS ${VISUALIZATION_DIR})

set(OUTPUT_DIR ${UTILS_DIR}/output)
list(APPEND LIST_INCLUDE_DIRS ${OUTPUT_DIR})


set(ALL_H_FILES
        ${SYSTEM_DIR}/System.hpp
        ${UTILS_DT_CONV_DIR}/YamlParserCV.hpp
        ${UTILS_DT_CONV_DIR}/DataConversion.hpp
        ${PARAMS_DIR}/Parameter.hpp
        ${PARAMS_DIR}/ParameterServer.hpp
        ${PARAMS_DIR}/ParameterBlueprint.h
        ${MESSAGING_DIR}/Message.hpp
        ${MESSAGING_DIR}/Interface.hpp
        ${DATA_LOADERS_DIR}/DataStore.hpp
        ${DATA_LOADERS_DIR}/TabularTextDS.hpp
        ${SENSOR_CAM_DIR}/Calibration.hpp
        ${SENSOR_CAM_DIR}/Camera.hpp
        ${SENSOR_CAM_MODELS_DIR}/GeometricCamera.h
        ${SENSOR_CAM_MODELS_DIR}/Pinhole.hpp
        ${SENSOR_CAM_MODELS_DIR}/PinholeRadTan.hpp
        ${SENSOR_CAM_MODELS_DIR}/KannalaBrandt8.hpp
        ${SENSOR_DATA_DIR}/SensorData.hpp
        ${SENSOR_DATA_DIR}/Image.hpp
        ${POSE_DIR}/Pose.hpp
        ${SENSORS_DIR}/Sensor.hpp
        ${POSE_DIR}/PoseProvider.hpp
        ${ATLAS_DIR}/Atlas.hpp
        ${ATLAS_DIR}/Map.hpp
        ${BACKEND_DIR}/BackEnd.hpp
        ${BACKEND_DIR}/Problem.hpp
        ${BACKEND_DIR}/BE_CalibCamCv.hpp
        ${BACKEND_DIR}/BE_GraphOptim.hpp
        ${OBSERVATIONS_DIR}/Observation.hpp
        ${OBSERVATIONS_DIR}/Point2D.hpp
        ${OBSERVATIONS_DIR}/MatchedFeatures.hpp
        ${OBSERVATIONS_DIR}/FeatureGrid.hpp
        ${WORLD_OBJ_DIR}/WorldObject.hpp
        ${WORLD_OBJ_DIR}/Point3D.hpp
        ${FRONTEND_DIR}/FrontEnd.hpp
        ${FRONTEND_DIR}/FE_CalibCamCv.hpp
        ${FRONTEND_DIR}/FE_ObjTracking.hpp
        ${FRONTEND_DIR}/FE_SlamMonoV.hpp
        ${OPERATORS_DIR}/Operator.hpp
        ${MAP_INIT_DIR}/OP_MapInitialization.hpp
        ${MAP_INIT_DIR}/OP_2ViewReconstruction.hpp
        ${OBJ_DET_DIR}/OP_ChBoardDetCv.hpp
        ${OBJ_DET_DIR}/OP_FtDtOrbSlam.hpp
        ${OBJ_DET_DIR}/OP_FtDt.hpp
        ${OBJ_ASSOC_DIR}/OP_FtAssoc.hpp
        ${OBJ_ASSOC_DIR}/OP_FtAssocOrbSlam.hpp
        ${OBJ_TRACK_DIR}/OP_ObjTracking.hpp
        ${OBJ_TRACK_DIR}/OP_ObjTrackingYolo.hpp
        ${OBJ_TRACK_DIR}/OP_ObjTrackingYoloPy.hpp
        ${OBJ_TRACK_DIR}/OP_ObjTrackingCv.hpp
        ${TRAJECTORY_DIR}/Trajectory.hpp
        ${TRAJECTORY_DIR}/TrajManager.hpp
        ${FRAME_DIR}/Frame.hpp
        ${DATA_TYPES_DIR}/SmartObject.hpp
        ${OPTIM_DT_DIR}/OptimizableTypes.hpp
#        ${OPTIM_DT_DIR}/G2oTypes.h
        ${OUTPUT_DIR}/Output.hpp
        ${OUTPUT_DIR}/Serial.hpp
        ${OUTPUT_DIR}/ImageViewer.hpp
        ${OUTPUT_DIR}/Visualization.hpp
        ${OUTPUT_DIR}/MapViewer.hpp
)
set(ALL_SRC_FILES
        ${SYSTEM_DIR}/System.cpp
        ${UTILS_DT_CONV_DIR}/YamlParserCV.cpp
        ${UTILS_DT_CONV_DIR}/DataConversion.cpp
        ${PARAMS_DIR}/Parameter.cpp
        ${PARAMS_DIR}/ParameterServer.cpp
        ${MESSAGING_DIR}/Message.cpp
        ${MESSAGING_DIR}/Interface.cpp
        ${DATA_LOADERS_DIR}/DataStore.cpp
        ${DATA_LOADERS_DIR}/TabularTextDS.cpp
        ${SENSOR_CAM_DIR}/Calibration.cpp
        ${SENSOR_CAM_DIR}/Camera.cpp
        ${SENSOR_CAM_MODELS_DIR}/Pinhole.cpp
        ${SENSOR_CAM_MODELS_DIR}/PinholeRadTan.cpp
        ${SENSOR_CAM_MODELS_DIR}/KannalaBrandt8.cpp
        ${SENSORS_DIR}/Sensor.cpp
        ${POSE_DIR}/PoseProvider.cpp
        ${SENSOR_DATA_DIR}/SensorData.cpp
        ${SENSOR_DATA_DIR}/Image.cpp
        ${POSE_DIR}/Pose.cpp
        ${ATLAS_DIR}/Atlas.cpp
        ${ATLAS_DIR}/Map.cpp
        ${BACKEND_DIR}/BackEnd.cpp
        ${BACKEND_DIR}/Problem.cpp
        ${BACKEND_DIR}/BE_CalibCamCv.cpp
        ${BACKEND_DIR}/BE_GraphOptim.cpp
        ${OBSERVATIONS_DIR}/Observation.cpp
        ${OBSERVATIONS_DIR}/Point2D.cpp
        ${OBSERVATIONS_DIR}/MatchedFeatures.cpp
        ${OBSERVATIONS_DIR}/FeatureGrid.cpp
        ${WORLD_OBJ_DIR}/WorldObject.cpp
        ${WORLD_OBJ_DIR}/Point3D.cpp
        ${FRONTEND_DIR}/FrontEnd.cpp
        ${FRONTEND_DIR}/FE_CalibCamCv.cpp
        ${FRONTEND_DIR}/FE_ObjTracking.cpp
        ${FRONTEND_DIR}/FE_SlamMonoV.cpp
        ${OPERATORS_DIR}/Operator.cpp
        ${MAP_INIT_DIR}/OP_MapInitialization.cpp
        ${MAP_INIT_DIR}/OP_2ViewReconstruction.cpp
        ${OBJ_DET_DIR}/OP_ChBoardDetCv.cpp
        ${OBJ_DET_DIR}/OP_FtDtOrbSlam.cpp
        ${OBJ_DET_DIR}/OP_FtDt.cpp
        ${OBJ_ASSOC_DIR}/OP_FtAssoc.cpp
        ${OBJ_ASSOC_DIR}/OP_FtAssocOrbSlam.cpp
        ${OBJ_TRACK_DIR}/OP_ObjTracking.cpp
        ${OBJ_TRACK_DIR}/OP_ObjTrackingYolo.cpp
        ${OBJ_TRACK_DIR}/OP_ObjTrackingCv.cpp
        ${OBJ_TRACK_DIR}/OP_ObjTrackingYoloPy.cpp
        ${TRAJECTORY_DIR}/Trajectory.cpp
        ${TRAJECTORY_DIR}/TrajManager.cpp
        ${FRAME_DIR}/Frame.cpp
        ${DATA_TYPES_DIR}/SmartObject.cpp
        ${OPTIM_DT_DIR}/OptimizableTypes.cpp
#        ${OPTIM_DT_DIR}/G2oTypes.cc
        ${OUTPUT_DIR}/Output.cpp
        ${OUTPUT_DIR}/Serial.cpp
        ${OUTPUT_DIR}/ImageViewer.cpp
        ${OUTPUT_DIR}/Visualization.cpp
        ${OUTPUT_DIR}/MapViewer.cpp
)
