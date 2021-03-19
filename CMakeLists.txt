cmake_minimum_required(VERSION 3.17)
project(cpo)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-DTRACE)            # enables 'trace' debugging in RTKLIB

#find_package(Eigen3 REQUIRED)
find_package(serial REQUIRED)

set(RTK_LIB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/deps/RTKLIB)

add_library(rtklib ${RTK_LIB_DIRECTORY}/src/rtkcmn.c
        ${RTK_LIB_DIRECTORY}/src/rtcm.c
        ${RTK_LIB_DIRECTORY}/src/rtcm2.c
        ${RTK_LIB_DIRECTORY}/src/rtcm3.c
        ${RTK_LIB_DIRECTORY}/src/rtcm3e.c
        )

add_executable(cpo src/main.cpp src/CpoFrontEnd.cpp)
target_include_directories(cpo PUBLIC ${RTK_LIB_DIRECTORY}/src include)
target_link_libraries(cpo serial rtklib)

add_executable(replay_RTCM src/replay_rtcm.cpp)
target_link_libraries(replay_RTCM serial)