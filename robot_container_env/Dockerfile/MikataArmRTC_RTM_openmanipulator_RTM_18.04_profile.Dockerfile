# Base image: Ubuntu 18.04
FROM ubuntu:bionic

# Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set working directory
WORKDIR /workspace

# Add OpenRTM-aist repository and install packages
RUN rm -f /etc/apt/sources.list.d/openrtm.list \
    && echo "deb [trusted=yes] http://openrtm.org/pub/Linux/ubuntu bionic main" > /etc/apt/sources.list.d/openrtm.list \
    && apt-get update \
    && apt-get install -y \
    git \
    build-essential \
    cmake \
    python-is-python3 \
    python3-pip \
    python3-dev \
    openrtm-aist \
    openrtm-aist-dev \
    libboost-all-dev \
    libudev-dev \
    libomiorb4-dev \
    omniorb-namesever \
    omniid \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Clone and build repository: gitURL1
RUN git clone  https://github.com/ogata-lab-admin/MikataArmRTC.git \
    && cd MikataArmRTC \
    && git submodule update --init --recursive \
    && mkdir -p build && cd build \
    && cmake .. -DBUILD_DOCUMENTATION=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && make -j$(nproc)


# Enter the container with a login shell
CMD ["bash", "-l"]
