FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

WORKDIR /ros2_ws

# Install essential packages and ROS development tools
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
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# For OpenGL/RViz diagnostics
RUN apt-get update && apt-get install -y \
    xauth \
    x11-apps \
    mesa-utils

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc \
    && mkdir -p -m 0700 /run/user/"${USER_UID}" \
    && mkdir -p -m 0700 /run/user/"${USER_UID}"/gdm \
    && chown user:user /run/user/"${USER_UID}" \
    && chown user:user /ros2_ws \
    && chown user:user /run/user/"${USER_UID}"/gdm \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install some ROS 2 dependencies to create a cache layer
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
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
        ros-humble-diff-drive-controller \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-teleop-twist-keyboard \
        ros-humble-joy \
        ros-humble-teleop-twist-joy \

    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

ENV XDG_RUNTIME_DIR=/run/user/"${USER_UID}"
RUN echo "user soft rtprio 99" >> /etc/security/limits.conf
RUN echo "user hard rtprio 99" >> /etc/security/limits.conf

# Install OpenHaptics & Haptic Device Drivers
RUN cd /tmp && curl -o TouchDriver.tgz https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver2022_04_04.tgz --output tmp/TouchDriver2022_04_04.tgz && \
    tar -axf TouchDriver*.tgz && sudo cp TouchDriver2022_04_04/bin/Touch* /usr/bin && sudo cp TouchDriver2022_04_04/usr/lib/libPhantomIOLib42.so /usr/lib
    # && cd .. && rm -r TouchDriver*
    
RUN cd /tmp && curl -o openhaptics.tgz https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz && \
    tar -axf openhaptics.tgz && cd openhaptics_* && bash -c 'echo -e "y\nq\n" | bash -v ./install'
    # && cd .. && rm -r openhaptics*


# FIX echo -e "y\nq\n" | sudo /tmp/openhaptics_3.4-0-developer-edition-amd64/install

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libncurses5-dev zlib1g-dev freeglut3-dev libncurses5 ros-humble-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libudev-dev \
    udev \
    usbutils

# set the default user to the newly created user
USER $USERNAME

# Install the missing ROS 2 dependencies
COPY . /ros2_ws/src
RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws \
    && vcs import src < src/dependency.repos --recursive --skip-existing \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
       --skip-keys="franka_selfcollision gz_sim_vendor parallel_gripper_controller gz_plugin_vendor" \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && rm -rf /home/$USERNAME/.ros \
    && rm -rf src \
    && mkdir -p src

COPY ./franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]
WORKDIR /ros2_ws