cmake_minimum_required(VERSION 3.17)
project(cpo)

set(CMAKE_CXX_STANDARD 17)

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
find_package(serial REQUIRED)
#include_directories(${SERIAL_INCLUDE_DIR})

add_executable(cpo src/main.cpp)