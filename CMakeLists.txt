cmake_minimum_required(VERSION 3.17)
project(cpo)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-DTRACE)            # enables 'trace' debugging in RTKLIB

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(serial REQUIRED)

set(RTK_LIB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/deps/RTKLIB)

add_library(rtklib ${RTK_LIB_DIRECTORY}/src/rtkcmn.c
        ${RTK_LIB_DIRECTORY}/src/rtcm.c
        ${RTK_LIB_DIRECTORY}/src/rtcm2.c
        ${RTK_LIB_DIRECTORY}/src/rtcm3.c
        ${RTK_LIB_DIRECTORY}/src/rtcm3e.c
        ${RTK_LIB_DIRECTORY}/src/pntpos.c
        ${RTK_LIB_DIRECTORY}/src/ephemeris.c
        ${RTK_LIB_DIRECTORY}/src/sbas.c
        ${RTK_LIB_DIRECTORY}/src/ionex.c
        ${RTK_LIB_DIRECTORY}/src/preceph.c
        )

add_executable(cpo src/main.cpp src/CpoFrontEnd.cpp src/SatelliteObs.cpp)
target_include_directories(cpo PUBLIC ${RTK_LIB_DIRECTORY}/src ${EIGEN3_INCLUDE_DIR} include)
target_link_libraries(cpo serial rtklib)

add_executable(replay_RTCM src/replay_rtcm.cpp)
target_link_libraries(replay_RTCM serial)

ament_package()