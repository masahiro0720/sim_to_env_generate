# Base image: ros:noetic-ros-base
FROM ros:noetic-ros-base

# Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install required packages
RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    cmake \
    ros-noetic-catkin \
    ros-noetic-moveit \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone required repositories
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/dynamixel-workbench.git 
RUN git clone -b noetic https://github.com/ROBOTIS-GIT/DynamixelSDK.git 
RUN git clone https://github.com/rsdlab/MikataArm.git 
RUN git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git 

# Resolve dependencies
WORKDIR /root/catkin_ws
# Initialize rosdep
RUN rosdep init || true && rosdep update
RUN rosdep install -r -y -i --from-paths /root/catkin_ws/src

# Build (execute after sourcing ROS environment)
RUN catkin init && \
    bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build"


# Add environment variables to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set default working directory
WORKDIR /root/catkin_ws

# Enter the container with a login shell
CMD ["bash", "-l"]
