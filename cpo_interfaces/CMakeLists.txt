cmake_minimum_required(VERSION 3.14)
project(cpo_interfaces)

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
rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_SRC}
        DEPENDENCIES builtin_interfaces geometry_msgs
        )

ament_export_dependencies(
        rosidl_default_runtime
)

ament_package()
