cmake_minimum_required(VERSION 3.17)
project(cpo)

set(CMAKE_CXX_STANDARD 17)

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
find_package(serial REQUIRED)

set(RTK_LIB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/deps/RTKLIB)

add_executable(cpo src/main.cpp)
target_include_directories(cpo PUBLIC ${RTK_LIB_DIRECTORY}/src)
target_link_libraries(cpo serial)