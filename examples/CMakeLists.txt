
project(nav_examples)

set(LIB_NAV_NAME nav_core)

if (USE_PYTHON)
    find_package(Python COMPONENTS Interpreter Development)
endif (USE_PYTHON)
if (Python_FOUND)
    add_compile_definitions(LIB_PYTHON_FOUND=1)
endif (Python_FOUND)
include_directories(${Python_INCLUDE_DIRS})
#list(APPEND LINKLIBS ${Python_LIBRARIES})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)

#add_executable(test_messaging tests/messaging.cpp)
#target_link_libraries(test_messaging LINK_PUBLIC ${LIB_NAV_NAME})
#list(APPEND INSTALLABLE_EXE test_messaging)
#
#add_executable(cv_yaml_parser tests/cv_yaml_parser.cpp)
#target_link_libraries(cv_yaml_parser LINK_PUBLIC ${LIB_NAV_NAME})
#list(APPEND INSTALLABLE_EXE cv_yaml_parser)
#
#add_executable(param_server tests/param_server.cpp)
#target_link_libraries(param_server LINK_PUBLIC ${LIB_NAV_NAME})
#list(APPEND INSTALLABLE_EXE param_server)
#
#add_executable(data_loaders tests/data_provider.cpp)
#target_link_libraries(data_loaders LINK_PUBLIC ${LIB_NAV_NAME})
#list(APPEND INSTALLABLE_EXE data_loaders)
#
#add_executable(camera tests/camera.cpp)
#target_link_libraries(camera LINK_PUBLIC ${LIB_NAV_NAME})
#list(APPEND INSTALLABLE_EXE camera)
#
#add_executable(tabular_text_ds tests/tabular_text_ds.cpp)
#target_link_libraries(tabular_text_ds LINK_PUBLIC ${LIB_NAV_NAME})
#list(APPEND INSTALLABLE_EXE tabular_text_ds)

# Tests

if (USE_PANGOLIN)
add_executable(world_viewer tests/world_viewer.cpp)
target_link_libraries(world_viewer LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE world_viewer)
endif (USE_PANGOLIN)

if (USE_SERIAL)
add_executable(serial_test tests/serial_test.cpp)
target_link_libraries(serial_test LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE serial_test)
endif (USE_SERIAL)

if (Python_FOUND)
add_executable(python_api tests/python_api.cpp)
target_link_libraries(python_api LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE python_api)
endif(Python_FOUND)

# Robotic Arm

add_executable(aun_robo_arm roboticArm/aun_robo_arm.cpp)
target_link_libraries(aun_robo_arm LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE aun_robo_arm)

if (Python_FOUND)
add_executable(aun_robo_arm_py roboticArm/aun_robo_arm_py.cpp)
target_link_libraries(aun_robo_arm_py LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE aun_robo_arm_py)
endif(Python_FOUND)

add_executable(aun_cam_calib roboticArm/aun_cam_calib.cpp)
target_link_libraries(aun_cam_calib LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE aun_cam_calib)

if (USE_SERIAL)
add_executable(aun_serial_test roboticArm/aun_serial_test.cpp)
target_link_libraries(aun_serial_test LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE aun_serial_test)
endif (USE_SERIAL)

#install(TARGETS ${INSTALLABLE_EXE} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/bin)

# SLAM

add_executable(v_mono slam/v_mono.cpp)
target_link_libraries(v_mono LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE v_mono)