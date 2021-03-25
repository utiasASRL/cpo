cmake_minimum_required(VERSION 3.14)
project(cpo_backend)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lgmath REQUIRED)
find_package(steam REQUIRED)
find_package(Threads)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(cpo_interfaces)

file(GLOB_RECURSE STEAM_EXT ${CMAKE_CURRENT_SOURCE_DIR}/src/steam_extensions/*.cpp)
add_library(cpo_steam_extensions ${STEAM_EXT})
target_include_directories(cpo_steam_extensions PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/steam_extensions
        ${EIGEN3_INCLUDE_DIR}
        ${lgmath_INCLUDE_DIRS}
        ${steam_INCLUDE_DIRS})

add_executable(cpo_backend src/main.cpp src/CpoBackEnd.cpp src/steam_extensions/TdcpErrorEval.cpp)
ament_target_dependencies(cpo_backend
        Eigen3
        rclcpp
        cpo_interfaces
        lgmath
        #        steam              # this line causing /targ_PATH set to NOTFOUND error
        )
target_include_directories(cpo_backend PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        ${lgmath_INCLUDE_DIRS}
        ${steam_INCLUDE_DIRS}
        include)
target_link_libraries(cpo_backend
        steam
        cpo_steam_extensions
        ${CMAKE_THREAD_LIBS_INIT})

ament_package()
