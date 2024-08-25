# the name of the target operating system
set(CMAKE_SYSTEM_NAME Linux)
set(MUSL_ROOT /path/to/musl/root)

# which compilers to use for C and C++
#set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
#set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_C_COMPILER   arm-linux-gnueabihf-gcc-9)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++-9)
#set(CMAKE_C_COMPILER   ${MUSL_ROOT}/aarch64-linux-musl-cross/bin/aarch64-linux-musl-gcc)
#set(CMAKE_CXX_COMPILER ${MUSL_ROOT}/aarch64-linux-musl-cross/bin/aarch64-linux-musl-g++)

# where is the target environment located
set(CMAKE_FIND_ROOT_PATH  /mnt/raspi-aun)

# adjust the default behavior of the FIND_XXX() commands:
# search programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# search headers and libraries in the target environment
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# run this command from the build dir
# cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/CrossCompilingForArm.cmake -Ddependency_DIR=/path/dependency ..
