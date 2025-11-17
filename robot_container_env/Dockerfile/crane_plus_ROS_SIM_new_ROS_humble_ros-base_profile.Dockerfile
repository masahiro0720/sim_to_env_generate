# Base image: ros:humble-ros-base
FROM ros:humble-ros-base

# Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install required packages
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    python3-colcon-common-extensions \
    ros-gazebo-ros2-control \
    ros-humble-ros-gz-sim \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-moveit \
    ros-humble-control-msgs \
    ros-humble-vision-opencv \
    libopencv-dev \
    ros-humble-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /root/colcon_ws/src
WORKDIR /root/colcon_ws/src

# Clone required repositories
RUN git clone -b humble https://github.com/rt-net/crane_plus.git 
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git 

# Resolve dependencies
WORKDIR /root/colcon_ws
# Initialize rosdep
RUN apt-get update && rosdep update
RUN rosdep install -r -y -i --from-paths /root/colcon_ws/src

# Build (execute after sourcing ROS environment)
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"


# Add environment variables to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

# Set default working directory
WORKDIR /root/colcon_ws

# Enter the container with a login shell
CMD ["bash", "-l"]
