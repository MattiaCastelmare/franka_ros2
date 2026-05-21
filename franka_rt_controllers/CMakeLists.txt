cmake_minimum_required(VERSION 3.5)
project(franka_rt_controllers)

# ── C++17 (matches franka_example_controllers) ──────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Shared library ──────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/rt_velocity_blender_controller.cpp
  src/rt_torque_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  franka_msgs
)

# ── Plugin export (tells controller_manager where to find the plugin) ─
pluginlib_export_plugin_description_file(
  controller_interface rt_velocity_blender_controller.xml
)

# ── Install ─────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── CBFTorqueController ──────────────────────────────────────────────
add_library(cbf_torque_controller SHARED
  src/cbf_torque_controller.cpp)
target_include_directories(cbf_torque_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cbf_torque_controller
  rclcpp controller_interface hardware_interface
  pluginlib realtime_tools ament_index_cpp
  std_msgs sensor_msgs Eigen3)
target_link_libraries(cbf_torque_controller pinocchio::pinocchio)
pluginlib_export_plugin_description_file(
  controller_interface franka_rt_controllers.xml)
install(TARGETS cbf_torque_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ── Ament exports ───────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  franka_msgs
  ament_index_cpp
  pinocchio
)

ament_package()
