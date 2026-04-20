# Base image (Pinned by SHA256 Digest for Reproducibility)
FROM docker.io/library/ros:jazzy-ros-base@sha256:cf4a86b16954f9d2d09c2a632ed6c546e9dfd6e74ab8672cbcc09c0a6db8b21c

# --- OCI Multi-Architecture Support ---
ARG TARGETARCH
LABEL org.opencontainers.image.architecture="${TARGETARCH:-amd64}"
# OCI Annotations (Image Metadata)
LABEL org.opencontainers.image.authors="Shota Aoki, Atsushi Kuwagata, Yusuke Kato"
LABEL org.opencontainers.image.description="CRANE+ V2ロボット用のROS 2パッケージスイート。制御、記述、シミュレーション、およびMoveIt!設定ファイルが含まれています。"

# --------------------------------------

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install APT packages and sudo
RUN apt-get update && apt-get install -y \
    sudo \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libopencv-dev \
    libudev-dev \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    rclcpp \
    ros-jazzy-controller-manager \
    ros-jazzy-cv-bridge \
    ros-jazzy-dynamixel-sdk \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-gazebo-ros2-control \
    ros-jazzy-geometry-msgs \
    ros-jazzy-gripper-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-hardware-interface \
    ros-jazzy-ign-ros2-control \
    ros-jazzy-image-geometry \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-moveit-configs-utils \
    ros-jazzy-moveit-core \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-ros-warehouse \
    ros-jazzy-moveit-setup-assistant \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-pluginlib \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2controlcli \
    ros-jazzy-rviz-common \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-usb-cam \
    ros-jazzy-vision-opencv \
    ros-jazzy-xacro \
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
    echo "if [ -f /workspace/colcon_ws/install/setup.bash ]; then source /workspace/colcon_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc
# ---------------------------------------------------

# Setup Workspace
RUN sudo mkdir -p /workspace/colcon_ws/src \
    && sudo chown -R $USERNAME:$USERNAME /workspace
WORKDIR /workspace/colcon_ws/src

# Clone & Build
RUN git clone -b jazzy https://github.com/rt-net/crane_plus.git
RUN git clone -b main https://github.com/ROBOTIS-GIT/DynamixelSDK.git

WORKDIR /workspace/colcon_ws
RUN sudo apt-get update && rosdep update && rosdep install -r -y -i --from-paths src
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install" 

# --- OCI Execution Control: ENTRYPOINT ---
# Create entrypoint script
RUN echo '#!/bin/bash' > /workspace/entrypoint.sh && \
    echo 'set -e' >> /workspace/entrypoint.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /workspace/entrypoint.sh && \
    echo 'if [ -f /workspace/colcon_ws/install/setup.bash ]; then source /workspace/colcon_ws/install/setup.bash; fi' >> /workspace/entrypoint.sh && \
    echo 'exec "$@"' >> /workspace/entrypoint.sh && \
    sudo chmod +x /workspace/entrypoint.sh

WORKDIR /workspace/colcon_ws

# --- OCI State Management: VOLUME ---
# Fix Errno 13: Pre-create the log directory and assign ownership to rosuser before mounting
RUN mkdir -p /home/$USERNAME/.ros/log     && sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/.ros
VOLUME ["/home/$USERNAME/.ros/log"]
# ------------------------------------

STOPSIGNAL SIGINT
ENTRYPOINT ["/workspace/entrypoint.sh"]
CMD ["bash"]
