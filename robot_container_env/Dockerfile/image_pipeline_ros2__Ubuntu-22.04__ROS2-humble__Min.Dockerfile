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
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libopencv-dev \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-setuptools \
    ros-humble-ament-cmake \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-image-transport \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-ros \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*


# Install Python dependencies
RUN pip3 install --no-cache-dir imutils


# Setup Workspace
RUN mkdir -p /root/colcon_ws/src
WORKDIR /root/colcon_ws/src

# Clone & Build
RUN git clone -b humble https://github.com/ros-perception/image_pipeline.git 

WORKDIR /root/colcon_ws
RUN apt-get update && rosdep update && rosdep install -r -y -i --from-paths src
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install" 

# Bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/colcon_ws
CMD ["bash", "-l"]
