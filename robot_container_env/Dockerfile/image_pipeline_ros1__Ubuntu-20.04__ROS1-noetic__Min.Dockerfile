# Base image
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install APT packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-utils \
    python3-catkin-tools \
    python3-opencv \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-noetic-camera-calibration-parsers \
    ros-noetic-camera-info-manager \
    ros-noetic-cv-bridge \
    ros-noetic-eigen-conversions \
    ros-noetic-image-geometry \
    ros-noetic-image-transport \
    ros-noetic-rospy \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-tf2 \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*


# Install Python dependencies
RUN pip3 install --no-cache-dir imutils


# Setup Workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone & Build
RUN git clone -b noetic https://github.com/ros-perception/image_pipeline.git 

WORKDIR /root/catkin_ws
RUN rosdep init || true && rosdep update && rosdep install -r -y -i --from-paths src
RUN catkin init && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build" 

# Bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/catkin_ws
CMD ["bash", "-l"]
