# Base image
FROM ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install APT packages
RUN apt-get update && apt-get install -y \
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
    ros-humble-controller-manager \
    ros-humble-cv-bridge \
    ros-humble-dynamixel-sdk \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-geometry-msgs \
    ros-humble-gripper-controllers \
    ros-humble-gz-ros2-control \
    ros-humble-hardware-interface \
    ros-humble-ign-ros2-control \
    ros-humble-image-geometry \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-core \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-ros-warehouse \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-pluginlib \
    ros-humble-rclcpp \
    ros-humble-robot-state-publisher \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros2controlcli \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz2 \
    ros-humble-tf2 \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-usb-cam \
    ros-humble-vision-opencv \
    ros-humble-xacro \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*



# Setup Workspace
RUN mkdir -p /root/colcon_ws/src
WORKDIR /root/colcon_ws/src

# Clone & Build
RUN git clone -b humble https://github.com/rt-net/crane_plus.git 
RUN git clone -b main https://github.com/ROBOTIS-GIT/DynamixelSDK.git 

WORKDIR /root/colcon_ws
RUN apt-get update && rosdep update && rosdep install -r -y -i --from-paths src
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install" 

# Bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/colcon_ws
CMD ["bash", "-l"]
