cmake_minimum_required(VERSION 3.16)
project(cpo_backend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-march=native -O3 -Wall -Wno-long-long -pedantic -Wno-unused-function)

add_definitions(-DSTEAM_DEFAULT_NUM_OPENMP_THREADS=1)       # can be used to make deterministic

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
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
ament_target_dependencies(cpo_steam_extensions
        Eigen3
        lgmath
        steam
        )
target_include_directories(cpo_steam_extensions
        PUBLIC
        cpo_steam_extensions
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/steam_extensions>
        $<INSTALL_INTERFACE:include>)


install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS cpo_steam_extensions
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

add_executable(cpo_backend src/main.cpp src/CpoBackEnd.cpp src/steam_extensions/TdcpErrorEval.cpp)
ament_target_dependencies(cpo_backend
        Eigen3
        rclcpp
        cpo_interfaces
        lgmath
        steam
        )
target_link_libraries(cpo_backend
        cpo_steam_extensions
        ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(cpo_backend
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS
        cpo_backend
        DESTINATION lib/${PROJECT_NAME})

# Parameter files
install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()
