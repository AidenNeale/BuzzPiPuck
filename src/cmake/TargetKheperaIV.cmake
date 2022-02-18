#
# Use this file as follows:
# $ cmake -DCMAKE_TOOLCHAIN_FILE=TargetKheperaIV.cmake <other args...>
#

# The name of the target system
# 'Linux' here is fine because the target board has a Linux OS on it
# NOTE: When this variable is set, the variable CMAKE_CROSSCOMPILING
# is also set automatically by CMake
set(CMAKE_SYSTEM_NAME Linux)

# Full path to C compiler
set(CMAKE_C_COMPILER /usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-gcc)

# Full path to C++ compiler
set(CMAKE_CXX_COMPILER /usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-g++)

# Set the root directories for the find_* commands
# Configure CMake to search for headers and libraries only in those directories
set(CMAKE_FIND_ROOT_PATH
  /usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr
  /usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr/include
  /usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr/lib)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# By default, install stuff in the toolchain tree
set(CMAKE_INSTALL_PREFIX /usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr CACHE STRING "Install path prefix, prepended onto install directories.")

# Compile with optimizations
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build")
