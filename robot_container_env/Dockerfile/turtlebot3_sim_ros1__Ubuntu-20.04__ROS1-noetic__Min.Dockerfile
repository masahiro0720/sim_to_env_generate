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
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros \
    ros-noetic-rospy \
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-turtlebot3-bringup \
    ros-noetic-turtlebot3-description \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3-teleop \
    ros-noetic-xacro \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*



# Setup Workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone & Build
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git 

WORKDIR /root/catkin_ws
RUN rosdep init || true && rosdep update && rosdep install -r -y -i --from-paths src
RUN catkin init && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build" 

# Bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/catkin_ws
CMD ["bash", "-l"]
