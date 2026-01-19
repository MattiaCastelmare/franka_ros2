FROM ros:humble-ros-base

# ------------------------
# Environment
# ------------------------
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# ------------------------
# Base system + vision deps
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
        python3-numpy \
        python3-opencv \
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
        portaudio19-dev \
        libasound2-dev \
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

USER $USERNAME

# ------------------------
# ROS 2 stack + RealSense
# ------------------------
RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-sdformat-urdf \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-controller-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-controller-manager \
        ros-humble-ros2-control-test-assets \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
        ros-humble-hardware-interface \
        ros-humble-control-msgs \
        ros-humble-backward-ros \
        ros-humble-generate-parameter-library \
        ros-humble-realtime-tools \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-moveit-servo \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-ros-testing \
        ros-humble-ros2test \
        ros-humble-ros2-control \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
        freeglut3-dev \
    && sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# ------------------------
# Python environment (MediaPipe FIX)
# ------------------------
ENV PATH=/home/${USERNAME}/.local/bin:$PATH
ENV PYTHONPATH=/home/${USERNAME}/.local/lib/python3.10/site-packages:$PYTHONPATH

RUN pip3 install --user --upgrade pip setuptools wheel && \
    pip3 install --user \
        protobuf>=4.25.3,<5 \
        absl-py \
        flatbuffers \
        sentencepiece \
        sounddevice \
        numpy==1.26.4 \
        opencv-contrib-python==4.11.0.86 && \
    pip3 install --user mediapipe==0.10.20

# ------------------------
# Workspace
# ------------------------
WORKDIR /ros2_ws
COPY . /ros2_ws/src

RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws && \
    vcs import src < src/franka.repos --recursive --skip-existing && \
    vcs import src < src/extras.repos --recursive --skip-existing && \
    sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys=pymoveit2 && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
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

SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]
