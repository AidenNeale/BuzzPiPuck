cmake_minimum_required(VERSION 3.5)
project(MY_PIPUCK LANGUAGES C CXX)
set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake ${CMAKE_INSTALL_PREFIX}/share/buzz/cmake)

find_package(Buzz REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIRS} )
message("Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set (CMAKE_PREFIX_PATH "/usr/include/")
# target_include_directories(asio INTERFACE /usr/include/asio)
find_package(ASIO REQUIRED)
add_definitions(-DASIO_STANDALONE)
add_definitions(-DASIO_HAS_STD_ADDRESSOF)
add_definitions(-DASIO_HAS_STD_ARRAY)
add_definitions(-DASIO_HAS_CSTDINT)
add_definitions(-DASIO_HAS_STD_SHARED_PTR)
add_definitions(-DASIO_HAS_STD_TYPE_TRAITS)

# set(MSP_SOURCE_DIR buzz/src/src_fc)
# set(MSP_INCLUDE_DIR buzz/include/inc_fc/msp)
# include_directories(${MSP_INCLUDE_DIR})



include_directories(${CMAKE_SOURCE_DIR}/buzz ${BUZZ_C_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/buzz ${KHEPERA_INCLUDE_DIR} ${BUZZ_C_INCLUDE_DIR})
link_directories(${KHEPERA_LIBRARY_DIR})

# client library
# add_library(mspclient ${MSP_SOURCE_DIR}/Client.cpp ${MSP_SOURCE_DIR}/PeriodicTimer.cpp)
# target_link_libraries(mspclient ${CMAKE_THREAD_LIBS_INIT} ASIO::ASIO)

add_executable(bzzPuck
  buzz/buzz_utility.c buzz/buzz_utility.h
  buzz/buzzpipuck_closures.c buzz/buzzpipuck_closures.h
  buzz/pipuck_utility.c buzz/pipuck_utility.h
  buzz/fc_inav.cpp                              buzz/fc_inav.h
  buzz/bzzpuck.c)
target_link_libraries(bzzPuck PUBLIC ${EIGEN3_LIBRARY} ${BUZZ_LIBRARY} ${BUZZ_LIBRARY_DEBUG} -lpthread -lm)

install(TARGETS bzzPuck RUNTIME DESTINATION bin)
