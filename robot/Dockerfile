FROM ros:humble-ros-base

# ============================================================
# Environment
# ============================================================

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

WORKDIR /robot_ws


# ============================================================
# Basic development tools
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        iputils-ping \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
        libgtest-dev \
        libgmock-dev \
        clang-14 \
        clang-format-14 \
        clang-tidy-14 \
        python3-pip \
        libpoco-dev \
        libeigen3-dev \
        libncurses5-dev \
        zlib1g-dev \
        freeglut3-dev \
        libncurses5 \
        libudev-dev \
        udev \
        usbutils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# X11 / OpenGL / RViz
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        xauth \
        x11-apps \
        mesa-utils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# User
# ============================================================

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && mkdir -p -m 0700 /run/user/"${USER_UID}" \
    && mkdir -p -m 0700 /run/user/"${USER_UID}"/gdm \
    && chown -R $USERNAME:$USERNAME /run/user/"${USER_UID}" \
    && chown $USERNAME:$USERNAME /robot_ws


# ============================================================
# ROS 2 dependencies
# ============================================================

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-ros-gz-interfaces \
        ros-humble-sdformat-urdf \
        ros-humble-gz-ros2-control \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-ros2-control \
        ros-humble-controller-interface \
        ros-humble-controller-manager \
        ros-humble-hardware-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ros2-control-test-assets \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-control-msgs \
        ros-humble-backward-ros \
        ros-humble-generate-parameter-library \
        ros-humble-realtime-tools \
        ros-humble-joint-state-broadcaster \
        ros-humble-diff-drive-controller \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-teleop-twist-keyboard \
        ros-humble-joy \
        ros-humble-teleop-twist-joy \
        ros-humble-librealsense2 \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-camera-msgs \
        ros-humble-realsense2-description \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Python development dependencies
# ============================================================

RUN python3 -m pip install -U \
        argcomplete \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-docstrings \
        flake8-import-order \
        flake8-quotes \
        pytest-repeat \
        pytest-rerunfailures \
        pytest


# ============================================================
# libfranka 0.13.3
# ============================================================

ARG LIBFRANKA_VERSION=0.13.3

RUN mkdir -p /opt/source_code \
    && cd /opt/source_code \
    && git clone https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && git checkout ${LIBFRANKA_VERSION} \
    && git submodule update --init --recursive \
    && mkdir build \
    && cd build \
    && cmake \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTS=OFF \
        .. \
    && make franka -j$(nproc) \
    && cpack -G DEB \
    && dpkg -i libfranka*.deb \
    && ldconfig


# ============================================================
# Environment configuration
# ============================================================

ENV XDG_RUNTIME_DIR=/run/user/"${USER_UID}"

RUN echo "user soft rtprio 99" >> /etc/security/limits.conf \
    && echo "user hard rtprio 99" >> /etc/security/limits.conf


# ============================================================
# Shell configuration
# ============================================================

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc


# ============================================================
# Workspace
# ============================================================

USER $USERNAME

COPY . /robot_ws/src

RUN sudo chown -R $USERNAME:$USERNAME /robot_ws \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro $ROS_DISTRO \
        -y \
        --skip-keys="franka_selfcollision gz_sim_vendor parallel_gripper_controller gz_plugin_vendor" \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && rm -rf /home/$USERNAME/.ros \
    && rm -rf src \
    && mkdir -p src

SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]
WORKDIR /robot_ws