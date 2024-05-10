
project(nav_examples)

set(LIB_NAV_NAME nav_core)



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



add_executable(aun_robo_arm roboticArm/aun_robo_arm.cpp)
target_link_libraries(aun_robo_arm LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE aun_robo_arm)

add_executable(aun_cam_calib roboticArm/aun_cam_calib.cpp)
target_link_libraries(aun_cam_calib LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE aun_cam_calib)

#install(TARGETS ${INSTALLABLE_EXE} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/bin)

