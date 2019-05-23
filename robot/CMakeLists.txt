cmake_minimum_required(VERSION 3.5)

project(robot)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED)


# msg srv action -----------------------------------------------------------


set(msg_files
        "src/msg/MoveBaseGoal.msg"
        "src/msg/MoveBaseActionGoal.msg"
        "src/msg/Contact.msg"
        )

set(srv_files
        "src/srv/AddTwoFloats.srv")

set(action_files
        "src/action/Fibonacci.action")


#rosidl_generate_interfaces(${PROJECT_NAME}
#        ${msg_files}
#        ${srv_files}
#        ${action_files}
#        DEPENDENCIES std_msgs geometry_msgs actionlib_msgs action_msgs builtin_interfaces
#        )

#ament_export_dependencies(rosidl_default_runtime)


# sensor -----------------------------------------------------------


add_executable(robot_laser src/laser/laser.cpp)
ament_target_dependencies(robot_laser
        "rclcpp"
        "sensor_msgs"
        )

add_executable(robot_cam2image src/camera/burger.cpp src/camera/cam2image.cpp)
ament_target_dependencies(robot_cam2image
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "OpenCV")

add_executable(robot_showimage src/camera/showimage.cpp)
ament_target_dependencies(robot_showimage
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "OpenCV")


# joint -----------------------------------------------------------


add_executable(robot_joint_states src/joint/joint_states.cpp)
ament_target_dependencies(robot_joint_states
        "rclcpp"
        "sensor_msgs")

add_executable(robot_state_publisher src/state/state_publisher.cpp)
ament_target_dependencies(robot_state_publisher
        "rclcpp"
        "sensor_msgs")


# map -----------------------------------------------------------


add_executable(robot_map_server src/map/map_server.cpp)
ament_target_dependencies(robot_map_server
        "rclcpp"
        "nav_msgs"
        )


# odometry -----------------------------------------------------------


add_executable(robot_odometry_publisher src/odometry/odometry_publisher.cpp)
ament_target_dependencies(robot_odometry_publisher
        "rclcpp"
        "tf2_ros"
        "nav_msgs")


#add_executable(simple_navigation_goals src/navigation/simple_navigation_goals.cpp)
#ament_target_dependencies(simple_navigation_goals
#        "rclcpp"
#        "rclcpp_action"
#        "tf2"
#        "action_msgs")


# lifecycle -----------------------------------------------------------


add_executable(lifecycle_talker src/lifecycle/lifecycle_talker.cpp)
ament_target_dependencies(lifecycle_talker
        "rclcpp"
        "rcutils"
        "std_msgs"
        "rclcpp_lifecycle"
        "lifecycle_msgs")

add_executable(lifecycle_listener src/lifecycle/lifecycle_listener.cpp)
ament_target_dependencies(lifecycle_listener
        "rclcpp"
        "rcutils"
        "std_msgs"
        "lifecycle_msgs")

add_executable(lifecycle_service_client src/lifecycle/lifecycle_service_client.cpp)
ament_target_dependencies(lifecycle_service_client
        "rclcpp"
        "rcutils"
        "lifecycle_msgs")


# install -----------------------------------------------------------


#install(TARGETS
#        robot_map_server
#        DESTINATION lib/${PROJECT_NAME})

install(
        PROGRAMS
        src/bin/fibonacci_action_client.py
        src/bin/fibonacci_action_server.py
        src/launch/robot_bringup.launch.py
        DESTINATION launch/${PROJECT_NAME}
)


#install(DIRECTORY launch
#        DESTINATION share/${PROJECT_NAME}/
#        )


# test ---------------------------------------------------------------


add_executable(talker src/test/talker.cpp)
ament_target_dependencies(talker
        "rclcpp"
        "std_msgs")
#install(TARGETS talker DESTINATION lib/${PROJECT_NAME})

add_executable(listener src/test/listener.cpp)
ament_target_dependencies(listener
        "rclcpp"
        "std_msgs")

add_executable(add_two_ints_service src/test/add_two_ints_service.cpp)
ament_target_dependencies(add_two_ints_service
        "rclcpp"
        "example_interfaces"
        )

add_executable(add_two_ints_client src/test/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client
        "rclcpp"
        "example_interfaces"
        )


add_executable(fibonacci_action_server src/test/fibonacci_action_server.cpp)
ament_target_dependencies(fibonacci_action_server
        "rclcpp"
        "rclcpp_action"
        "example_interfaces"
        )

add_executable(fibonacci_action_client src/test/fibonacci_action_client.cpp)
ament_target_dependencies(fibonacci_action_client
        "rclcpp"
        "rclcpp_action"
        "example_interfaces"
        )

#add_executable(publish_contact src/test/publish_contact.cpp)
#ament_target_dependencies(publish_contact
#        "rclcpp")

# test -----------------------------------------------------------


#get_default_rmw_implementation(rmw_implementation)
#find_package("${rmw_implementation}" REQUIRED)
#get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
#
#foreach (typesupport_impl ${typesupport_impls})
#    rosidl_target_interfaces(publish_contact
#            ${PROJECT_NAME} ${typesupport_impl}
#            )
#
##        rosidl_target_interfaces(simple_navigation_goals
##                ${PROJECT_NAME} ${typesupport_impl}
##                )
#endforeach ()


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
