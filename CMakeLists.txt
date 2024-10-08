cmake_minimum_required(VERSION 3.22.1)
project(NAV24 VERSION 1.0)

#include(GNUInstallDirs)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(PROJ_CORE_PATH core)
set(PROJ_EXAMPLES_PATH examples)
set(PROJ_ROS_PATH ros)
set(PROJ_THIRDPARTY_PATH thirdparty)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/lib)
#set(DESTDIR ${CMAKE_INSTALL_PREFIX})


configure_file(${PROJ_CORE_PATH}/GlobalConfig.h.in ${PROJ_CORE_PATH}/GlobalConfig.h)

option(USE_PANGOLIN "Required for 3D data visualization" OFF)
option(USE_SERIAL "Required for RS232 serial communication (Arduino)" OFF)
option(USE_PYTHON "Use python library to embed python in C++" OFF)
option(USE_ONNX_RUNTIME "Use ONNX Runtime to enable deep learning in C++" OFF)

# External dependencies
add_subdirectory(${PROJ_THIRDPARTY_PATH}/g2o)
add_subdirectory(${PROJ_THIRDPARTY_PATH}/DBoW2)
if (USE_SERIAL)
add_subdirectory(${PROJ_THIRDPARTY_PATH}/serial)
endif()
#add_subdirectory(${PROJ_THIRDPARTY_PATH}/onnxruntime)

add_subdirectory(${PROJ_EXAMPLES_PATH})
add_subdirectory(${PROJ_CORE_PATH})
# WARNING: Always source ros setup before configuring this.
#add_subdirectory(${PROJ_ROS_PATH})


install(FILES ${PROJECT_BINARY_DIR}/${PROJ_CORE_PATH}/GlobalConfig.h
        DESTINATION include ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT NAV_CORE
        FILE NAV_CORE.cmake
        NAMESPACE NAV24::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake
)

include(CMakePackageConfigHelpers)
# generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/NAV_COREConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake
        NO_SET_AND_CHECK_MACRO
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
# generate the version file for the config file
write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/NAV_COREConfigVersion.cmake"
        VERSION "${NAV24_VERSION_MAJOR}.${NAV24_VERSION_MINOR}"
        COMPATIBILITY AnyNewerVersion
)

# install the configuration file
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/NAV_COREConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/NAV_COREConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake
)

export(EXPORT NAV_CORE
        FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/NAV_CORE.cmake"
)



