cmake_minimum_required(VERSION 3.8)
project(franka_simulation_experiments)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml")

# Nodes and tools, installed without the .py (same convention as franka_simulation).
file(GLOB PYTHON_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
foreach(script ${PYTHON_SCRIPTS})
  get_filename_component(script_name ${script} NAME_WE)
  install(PROGRAMS ${script} DESTINATION lib/${PROJECT_NAME} RENAME ${script_name})
endforeach()

# Pure logic used by the nodes: lib/franka_simulation_experiments/simexp/.
# A different name from franka_simulation's `utils` package, so a node can
# import both without one shadowing the other.
install(DIRECTORY scripts/simexp DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

ament_package()
