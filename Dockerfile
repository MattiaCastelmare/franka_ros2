FROM ros:humble-ros-base

# ============================================================================
# Build-time arguments and global environment
# ============================================================================
ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# ============================================================================
# System packages
#
# Base utilities + GUI/vision libs + ROS 2 stack + Ignition (Fortress)
# dev libs + AprilTag + Pinocchio (robotpkg).
#
# Everything is installed in a single layer (with the robotpkg source
# registered in-between the two apt-get update calls) so that:
#   * Docker only caches/uploads one big system layer.
#   * The whole apt cache is dropped at the end.
#   * Source-code changes never invalidate this layer.
# ============================================================================
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        freeglut3-dev \
        gdb \
        git \
        gnupg2 \
        libasound2-dev \
        libgl1 \
        libglib2.0-0 \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
        libsm6 \
        libxext6 \
        libxrender-dev \
        lsb-release \
        nano \
        openssh-client \
        portaudio19-dev \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        python3-dev \
        python3-numpy \
        python3-opencv \
        python3-pip \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-apriltag-ros \
        ros-humble-backward-ros \
        ros-humble-control-msgs \
        ros-humble-controller-interface \
        ros-humble-controller-manager \
        ros-humble-generate-parameter-library \
        ros-humble-hardware-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-joint-state-broadcaster \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-ros-visualization \
        ros-humble-moveit-servo \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-pymoveit2 \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
        ros-humble-realtime-tools \
        ros-humble-ros-gz \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
        ros-humble-ros-testing \
        ros-humble-ros2-control \
        ros-humble-ros2-control-test-assets \
        ros-humble-ros2controlcli \
        ros-humble-ros2test \
        ros-humble-rviz2 \
        ros-humble-sdformat-urdf \
        ros-humble-tf2-geometry-msgs \
        ros-humble-xacro \
        sudo \
        udev \
        usbutils \
        v4l-utils \
        vim \
 && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
        | gpg --dearmor -o /usr/share/keyrings/robotpkg.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/robotpkg.gpg] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
        > /etc/apt/sources.list.d/robotpkg.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
        robotpkg-py310-pinocchio \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# ============================================================================
# Pinocchio environment (set BEFORE the smoke-test below)
# ============================================================================
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH \
    PATH=/opt/openrobots/bin:$PATH \
    LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH \
    PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH

RUN python3 -c "import pinocchio as pin; print('pinocchio', pin.__version__)"

# ============================================================================
# Gazebo / Ignition environment
# ============================================================================
ENV GZ_VERSION=fortress \
    IGN_GAZEBO_RESOURCE_PATH=/usr/share/gazebo_models:$IGN_GAZEBO_RESOURCE_PATH \
    GZ_SIM_RESOURCE_PATH=/usr/share/gazebo_models:$GZ_SIM_RESOURCE_PATH

# ============================================================================
# Non-root user (UID/GID configurable via build-args)
#
# Passwordless sudo is granted via a drop-in file in /etc/sudoers.d/, which
# is the recommended location (per-user file, mode 0440).
# ============================================================================
RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd  --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash"                          >> /home/${USERNAME}/.bashrc \
 && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc

# Make user-local pip binaries visible to every subsequent shell / RUN.
ENV PATH=/home/${USERNAME}/.local/bin:$PATH

# ============================================================================
# User-space Python packages (MediaPipe stack + qpsolvers).
#
# Installed as the non-root user with --user so everything lives in
# /home/${USERNAME}/.local and does not collide with system site-packages.
# Pinned versions reproduce the working MediaPipe stack.
# --no-cache-dir keeps the layer small.
#
# VERSION LOCK — qpsolvers 4.3.3 + osqp 0.6.x, do not float either:
#   * osqp must stay on 0.6.x because cbf_safety_filter.py and
#     franka_sim/envs/cbf_filter.py drive the RAW OSQP object
#     (`OSQP().setup()/.update()`, `res.info.status_val`); osqp 1.x rewrote
#     that API and would break the safety filter.
#   * qpsolvers ≥ 4.4 imports `from osqp import OSQP, SolverStatus`, which only
#     exists in osqp 1.x. With an unpinned qpsolvers the osqp backend silently
#     fails to load (`available_solvers == []`) and cbf_velocity_filter /
#     cbf_OSCBF_filter raise SolverNotFound at their first QP.
# 4.3.3 is the last release that speaks the osqp 0.6 API.
# ============================================================================
USER ${USERNAME}

RUN python3 -m pip install --user --no-cache-dir --upgrade \
        "pip" \
        "setuptools<80" \
        "wheel" \
 && python3 -m pip install --user --no-cache-dir \
        "protobuf>=4.25.3,<5" \
        absl-py \
        flatbuffers \
        sentencepiece \
        sounddevice \
        numpy==1.26.4 \
        opencv-contrib-python==4.11.0.86 \
        trimesh \
 && python3 -m pip install --user --no-cache-dir \
        mediapipe==0.10.20 \
 && python3 -m pip install --user --no-cache-dir \
        "qpsolvers[osqp]==4.3.3" \
        "osqp>=0.6.2,<1.0"

# ============================================================================
# Safe-RL + CBF Sim-to-Real training stack (franka_sim/).
#
# MuJoCo physics + Gymnasium + Stable-Baselines3 (SAC) on CUDA, plus the ONNX
# export/inference runtime used by the deployment node. Pinned to the versions
# validated for the RTX 4070 (torch bundles its own CUDA runtime → GPU comes
# from the container's `gpus: all` + NVIDIA driver, no system CUDA needed).
#
# IMPORTANT: onnx pulls a protobuf 7.x that BREAKS the mediapipe stack above
# (mediapipe needs protobuf <5). protobuf is therefore re-pinned to 4.25.x as
# the LAST step of this layer so both stacks coexist. The CBF QP uses raw osqp
# (already provided by qpsolvers[osqp]), exactly like cbf_safety_filter.py.
# ============================================================================
RUN python3 -m pip install --user --no-cache-dir \
        torch==2.13.0 \
        mujoco==3.4.0 \
        gymnasium==0.29.1 \
        stable-baselines3==2.9.0 \
        tensorboard==2.16.2 \
        onnx==1.22.0 \
        onnxruntime==1.23.2 \
        tqdm \
        rich \
 && python3 -m pip install --user --no-cache-dir \
        "protobuf>=4.25.3,<5"

# ============================================================================
# Python environment smoke test.
#
# Fails the build if any core import is broken. For OpenCV the loaded
# version and module path are printed so it is visible whether the pip
# wheel (~/.local, opencv-contrib 4.11) or the apt python3-opencv
# (/usr/lib/python3/dist-packages) is being picked up.
# ============================================================================
RUN python3 -c "\
import pinocchio, cv2, mediapipe, qpsolvers; \
print('pinocchio', pinocchio.__version__); \
print('cv2      ', cv2.__version__, '<-', cv2.__file__); \
print('mediapipe OK'); \
assert 'osqp' in qpsolvers.available_solvers, \
    'qpsolvers cannot load its osqp backend (version skew) — the velocity/OSCBF ' \
    'filters would raise SolverNotFound at runtime'; \
print('qpsolvers', qpsolvers.__version__, qpsolvers.available_solvers)" \
 && python3 -c "\
import torch, mujoco, gymnasium, stable_baselines3, onnx, onnxruntime, osqp; \
from torch.utils.tensorboard import SummaryWriter; \
import mediapipe, google.protobuf as pb; \
print('torch    ', torch.__version__, 'cuda-build', torch.version.cuda); \
print('mujoco   ', mujoco.__version__); \
print('gymnasium', gymnasium.__version__); \
print('sb3      ', stable_baselines3.__version__); \
print('onnxrt   ', onnxruntime.__version__); \
print('protobuf ', pb.__version__, '(tensorboard SummaryWriter + mediapipe coexist)'); \
print('osqp OK — franka_sim training stack ready')"

# ============================================================================
# Entrypoint script (root-owned, executable)
# ============================================================================
USER root
COPY ./franka_entrypoint.sh /franka_entrypoint.sh
RUN chmod +x /franka_entrypoint.sh

# ============================================================================
# Workspace bootstrap.
#
# Split in two stages for caching:
#   1. Only franka.repos is copied and the external repositories are
#      cloned. Edits to the project sources do not invalidate this layer,
#      so the GitHub downloads are not repeated on every code change.
#   2. The full sources are copied on top (local checkouts of the same
#      repos, if present in the context, simply overlay the clones) and
#      rosdep resolves the dependencies of the whole tree.
#
# vcs + rosdep run as root (no sudo gymnastics), then the embedded
# sources are wiped and ownership of /ros2_ws is handed to the runtime
# user.
# ============================================================================
WORKDIR /ros2_ws

COPY ./franka.repos /tmp/franka.repos
RUN mkdir -p src \
 && vcs import src < /tmp/franka.repos --recursive --skip-existing

COPY . /ros2_ws/src

RUN apt-get update \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --skip-keys ament_python \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && rm -rf /root/.ros \
 && rm -rf src \
 && mkdir -p src \
 && chown -R ${USERNAME}:${USERNAME} /ros2_ws

# ============================================================================
# Runtime
# ============================================================================
USER ${USERNAME}
SHELL ["/bin/bash", "-c"]
ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]
