cmake_minimum_required(VERSION 3.14)
project(cpo_frontend)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-DTRACE)            # enables 'trace' debugging in RTKLIB

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(serial REQUIRED)

find_package(cpo_interfaces)

set(RTK_LIB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../deps/RTKLIB)

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

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS rtklib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

add_executable(cpo_frontend src/main.cpp src/CpoFrontEnd.cpp src/SatelliteObs.cpp)
ament_target_dependencies(cpo_frontend rclcpp cpo_interfaces)
target_include_directories(cpo_frontend PUBLIC ${RTK_LIB_DIRECTORY}/src ${EIGEN3_INCLUDE_DIR} include)
target_link_libraries(cpo_frontend serial rtklib)

add_executable(replay_RTCM src/replay_rtcm.cpp)
target_include_directories(replay_RTCM PUBLIC include)
target_link_libraries(replay_RTCM serial)

add_executable(clock_server src/clock_server.cpp)
ament_target_dependencies(clock_server rclcpp)

install(TARGETS
        cpo_frontend
        replay_RTCM
        clock_server
        DESTINATION lib/${PROJECT_NAME})

# Parameter files
install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()