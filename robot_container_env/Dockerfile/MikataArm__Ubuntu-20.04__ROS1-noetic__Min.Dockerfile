# Base image (Pinned by SHA256 Digest for Reproducibility)
FROM docker.io/library/ros:noetic-ros-base@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d

# --- OCI Multi-Architecture Support ---
ARG TARGETARCH
LABEL org.opencontainers.image.architecture="${TARGETARCH:-amd64}"
# OCI Annotations (Image Metadata)
LABEL org.opencontainers.image.authors="Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na"
LABEL org.opencontainers.image.description="既存のMikataArmパッケージで見つかったMoveIt!との統合問題を解決したROS対応OpenManipulatorであり、実機環境とシミュレーション環境の両方でシームレスな操作を可能にするオープンロボットプラットフォームです。"

# --------------------------------------

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install APT packages and sudo
RUN apt-get update && apt-get install -y \
    sudo \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libeigen3-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-utils \
    python3-catkin-tools \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-noetic-actionlib \
    ros-noetic-cmake-modules \
    ros-noetic-dynamixel-workbench-toolbox \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-moveit \
    ros-noetic-moveit-core \
    ros-noetic-moveit-ros-planning \
    ros-noetic-moveit-ros-planning-interface \
    ros-noetic-robot-state-publisher \
    ros-noetic-robotis-manipulator \
    ros-noetic-rviz \
    ros-noetic-urdf \
    ros-noetic-xacro \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*



# --- OCI Security: Create non-root user (rosuser) ---
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME

# --- UX: Automatic source for Interactive Shell (exec) ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /workspace/catkin_ws/devel/setup.bash ]; then source /workspace/catkin_ws/devel/setup.bash; fi" >> /home/$USERNAME/.bashrc
# ---------------------------------------------------

# Setup Workspace
RUN sudo mkdir -p /workspace/catkin_ws/src \
    && sudo chown -R $USERNAME:$USERNAME /workspace
WORKDIR /workspace/catkin_ws/src

# Clone & Build
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
RUN git clone -b main https://github.com/rsdlab/MikataArm.git
RUN git clone -b main https://github.com/ROBOTIS-GIT/robotis_manipulator.git

WORKDIR /workspace/catkin_ws
RUN rosdep init || true && rosdep update && sudo apt-get update && rosdep install -r -y -i --from-paths src
RUN catkin init && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build" 

# --- OCI Execution Control: ENTRYPOINT ---
# Create entrypoint script
RUN echo '#!/bin/bash' > /workspace/entrypoint.sh && \
    echo 'set -e' >> /workspace/entrypoint.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /workspace/entrypoint.sh && \
    echo 'if [ -f /workspace/catkin_ws/devel/setup.bash ]; then source /workspace/catkin_ws/devel/setup.bash; fi' >> /workspace/entrypoint.sh && \
    echo 'exec "$@"' >> /workspace/entrypoint.sh && \
    sudo chmod +x /workspace/entrypoint.sh

WORKDIR /workspace/catkin_ws

# --- OCI State Management: VOLUME ---
# Fix Errno 13: Pre-create the log directory and assign ownership to rosuser before mounting
RUN mkdir -p /home/$USERNAME/.ros/log     && sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/.ros
VOLUME ["/home/$USERNAME/.ros/log"]
# ------------------------------------

STOPSIGNAL SIGINT
ENTRYPOINT ["/workspace/entrypoint.sh"]
CMD ["bash"]
