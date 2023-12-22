
project(nav_examples)

set(LIB_NAV_NAME nav_core)



set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)

add_executable(data_loaders tests/data_loaders.cpp)
target_link_libraries(data_loaders LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE data_loaders)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/roboticArm)

add_executable(aun_robo_arm roboticArm/aun_robo_arm.cpp)
target_link_libraries(aun_robo_arm LINK_PUBLIC ${LIB_NAV_NAME})
list(APPEND INSTALLABLE_EXE aun_robo_arm)

#install(TARGETS ${INSTALLABLE_EXE} DESTINATION ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/bin)

