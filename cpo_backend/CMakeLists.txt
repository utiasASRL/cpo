cmake_minimum_required(VERSION 3.14)
project(cpo_backend)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lgmath REQUIRED)
find_package(steam REQUIRED)

find_package(cpo_interfaces)

add_executable(cpo_backend src/main.cpp src/CpoBackEnd.cpp)
ament_target_dependencies(cpo_backend
        Eigen3
        rclcpp
        cpo_interfaces
        lgmath
#        steam              # this line causing /targ_PATH set to NOTFOUND error
        )
target_include_directories(cpo_backend PUBLIC ${EIGEN3_INCLUDE_DIR} ${lgmath_INCLUDE_DIRS} ${steam_INCLUDE_DIRS} include)
target_link_libraries(cpo_backend steam)


ament_package()
