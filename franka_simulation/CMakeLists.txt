cmake_minimum_required(VERSION 3.8)
project(franka_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Core ROS 2 packages
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# MoveIt dependencies
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_msgs REQUIRED)

# TF2 and transformations
find_package(tf2_ros REQUIRED)

# Action support - NUOVO per action custom
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ros2_control stack
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

# Franka dependencies
find_package(franka_description REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_fr3_moveit_config REQUIRED)

# Simulation dependencies
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Visualization
find_package(rviz2 REQUIRED)

# Generate action interfaces - NUOVO
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  "action/MoveToJoint.action"
  DEPENDENCIES 
    geometry_msgs 
    sensor_msgs 
    moveit_msgs
    std_msgs
)

# Make sure the action generation is available for the targets
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Create directories that will be needed
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/action)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

# Install Python scripts
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

# Install action definitions
install(DIRECTORY action/
  DESTINATION share/${PROJECT_NAME}/action
  FILES_MATCHING PATTERN "*.action"
)

# Install server and client as executables
install(PROGRAMS
  scripts/franka_motion_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_motion_server
)

install(PROGRAMS
  scripts/franka_motion_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_motion_client
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies for action interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()