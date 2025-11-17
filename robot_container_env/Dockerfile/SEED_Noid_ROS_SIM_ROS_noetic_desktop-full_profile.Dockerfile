# Base image: ros:noetic-desktop-full
FROM ros:noetic-desktop-full

# Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic-desktop-full

# Install required packages
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    ros-noetic-moveit \
    ros-noetic-moveit-visual-tools \
    ros-noetic-rviz \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros \
    ros-noetic-tf2-ros \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-plugins \
    ros-noetic-move-base-msgs \
    ros-noetic-robot-state-publisher \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-dwa-local-planner \
    ros-noetic-global-planner \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Clone required repositories
RUN git clone https://github.com/seed-solutions/seed_smartactuator_sdk.git 
RUN git clone https://github.com/seed-solutions/seed_r7_ros_pkg.git 

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
