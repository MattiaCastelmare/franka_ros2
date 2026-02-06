#!/bin/bash

# Clone Franka dependencies into the workspace (libfranka, franka_description)
vcs import /ros2_ws/src < /ros2_ws/src/franka.repos --recursive --skip-existing

# Clone pymoveit2 (lightweight Python MoveIt2 wrapper, not available via apt)
if [ ! -d "/ros2_ws/src/pymoveit2" ]; then
    git clone --depth 1 https://github.com/AndrejOrsula/pymoveit2.git /ros2_ws/src/pymoveit2
fi

exec "$@"