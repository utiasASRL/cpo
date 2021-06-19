cmake_minimum_required(VERSION 3.16)
project(cpo_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-march=native -O3 -pthread -Wall -Wextra)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# ROS messages
set(MSG_SRC
        "msg/TDCP.msg"
        "msg/SatPair.msg"
        )
set(SRV_SRC
        "srv/QueryTrajectory.srv"
        )
rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_SRC}
        ${SRV_SRC}
        DEPENDENCIES builtin_interfaces geometry_msgs
        )

ament_export_dependencies(
        rosidl_default_runtime
)

ament_package()
