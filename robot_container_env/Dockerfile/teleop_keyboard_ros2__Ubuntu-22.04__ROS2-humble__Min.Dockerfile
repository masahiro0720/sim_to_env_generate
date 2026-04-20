# Base image
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install APT packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-setuptools \
    ros-humble-geometry-msgs \
    ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/*



# Setup Workspace
RUN mkdir -p /root/colcon_ws/src
WORKDIR /root/colcon_ws/src

# Clone & Build
RUN git clone -b humble https://github.com/ros-teleop/teleop_twist_keyboard.git 

WORKDIR /root/colcon_ws
RUN apt-get update && rosdep update && rosdep install -r -y -i --from-paths src
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install" 

# Bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/colcon_ws
CMD ["bash", "-l"]
