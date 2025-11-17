# Base image: ros:humble-ros-base
FROM ros:humble-ros-base

# Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install required packages
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    udev \
    python3-opencv \
    python3-numpy \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /root/colcon_ws/src
WORKDIR /root/colcon_ws/src

# Clone required repositories

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
