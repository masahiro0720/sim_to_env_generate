# Base image
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install APT packages
RUN apt-get update && apt-get install -y \
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



# Setup Workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone & Build
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/DynamixelSDK.git 
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/dynamixel-workbench.git 
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git 
RUN git clone -b main https://github.com/rsdlab/MikataArm.git 
RUN git clone -b main https://github.com/ROBOTIS-GIT/robotis_manipulator.git 

WORKDIR /root/catkin_ws
RUN rosdep init || true && rosdep update && rosdep install -r -y -i --from-paths src
RUN catkin init && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build" 

# Bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/catkin_ws
CMD ["bash", "-l"]
