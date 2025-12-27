FROM ros:humble-ros-base

# ------------------------
# Environment
# ------------------------
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# ------------------------
# Ubuntu repositories
# ------------------------
# Some dependencies resolved by rosdep (e.g., libpoco-dev) live in Ubuntu "universe".
# NOTE: Using `add-apt-repository` inside minimal images is sometimes unreliable depending
# on the base image's apt source layout. This block makes "universe" enablement explicit
# and idempotent for both legacy and deb822 apt source formats.
RUN set -eux; \
    if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then \
        if grep -q '^Components:' /etc/apt/sources.list.d/ubuntu.sources && ! grep -q '^Components:.*\buniverse\b' /etc/apt/sources.list.d/ubuntu.sources; then \
            sed -i -E 's/^(Components:.*)$/\1 universe/' /etc/apt/sources.list.d/ubuntu.sources; \
        fi; \
    fi; \
    if [ -f /etc/apt/sources.list ]; then \
        sed -i -E '/^deb\s+.*\s+jammy(-updates|-security|-backports)?\s+/ { /\buniverse\b/! s/$/ universe/ }' /etc/apt/sources.list; \
        # De-duplicate sources to avoid noisy apt warnings in later layers.
        awk '!seen[$0]++' /etc/apt/sources.list > /tmp/sources.list && mv /tmp/sources.list /etc/apt/sources.list; \
    fi; \
    apt-get update; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/*

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# ------------------------
# Base system dependencies
# ------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        python3-pip \
        python3-dev \
        sudo \
        vim \
        usbutils \
        udev \
        v4l-utils \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender-dev \
        libstdc++6 \
        libgcc-s1 \
        libpoco-dev \
        qtbase5-dev \
        python3-numpy \
        python3-opencv \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ------------------------
# Create user
# ------------------------
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# ------------------------
# ROS vision
# ------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-cv-bridge \
        ros-humble-rviz2 \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ------------------------
# Switch to user
# ------------------------
USER $USERNAME

# ------------------------
# Python vision stack (SAFE)
# ------------------------
RUN pip3 install --no-cache-dir mediapipe==0.10.20 --no-deps

# ------------------------
# ROS / Gazebo / MoveIt stack
# ------------------------
USER root
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-sdformat-urdf \
        ros-humble-robot-state-publisher \
        ros-humble-tf2-ros \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-controller-interface \
        ros-humble-controller-manager \
        ros-humble-hardware-interface \
        ros-humble-control-msgs \
        ros-humble-realtime-tools \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-xacro \
        ros-humble-ros2-control \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

USER $USERNAME

# ------------------------
# Workspace
# ------------------------
ARG ROSDEP_SKIP_KEYS="\
pymoveit2 \
pinocchio \
gripper_controllers \
velocity_controllers \
position_controllers \
ros2_controllers \
moveit_resources_panda_moveit_config \
moveit_resources_panda_description \
moveit_resources_pr2_description \
moveit_resources_fanuc_description \
moveit_resources_fanuc_moveit_config \
ros_testing \
ament_clang_format \
ament_cmake_clang_format \
ament_cmake_clang_tidy \
launch_param_builder \
qt5-opengl-dev \
libqt5opengl5-dev"


WORKDIR /ros2_ws
COPY . /ros2_ws/src

RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws && \
    vcs import src < src/franka.repos --recursive --skip-existing && \
    rosdep update && \
    sudo apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
        --dependency-types build \
        --dependency-types buildtool \
        --dependency-types build_export \
        --dependency-types buildtool_export \
        --dependency-types exec \
        --skip-keys="$ROSDEP_SKIP_KEYS" && \
    rm -rf /home/$USERNAME/.ros && \
    rm -rf src && \
    mkdir -p src

# ------------------------
# Entrypoint
# ------------------------
COPY ./franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

# ------------------------
# Gazebo env
# ------------------------
ENV GZ_VERSION=fortress
ENV IGN_GAZEBO_RESOURCE_PATH=/usr/share/gazebo_models:${IGN_GAZEBO_RESOURCE_PATH}
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gazebo_models:${GZ_SIM_RESOURCE_PATH}

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/franka_entrypoint.sh"]
CMD ["/bin/bash"]
