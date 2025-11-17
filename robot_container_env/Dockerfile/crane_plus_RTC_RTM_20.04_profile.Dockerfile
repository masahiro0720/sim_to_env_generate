# Base image: Linux 20.04
FROM ubuntu:20.04

# Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set working directory
WORKDIR /workspace

# Add OpenRTM-aist repository and install packages
RUN rm -f /etc/apt/sources.list.d/openrtm.list \
    && echo "deb [trusted=yes] http://openrtm.org/pub/Linux/ubuntu focal main" > /etc/apt/sources.list.d/openrtm.list \
    && apt-get update \
    && apt-get install -y \
    ca-certificates \
    git \
    build-essential \
    cmake \
    doxygen \
    python-is-python3 \
    python3-pip \
    python3-dev \
    libomniorb4-dev \
    omniorb-nameserver \
    omniidl \
    libboost-all-dev \
    libudev-dev \
    openrtm-aist \
    openrtm-aist-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Clone and build repository: CraneplusRTC_ver2
RUN git clone  https://github.com/masahiro0720/CRANEplusRTC_ver2.git \
    && cd CRANEplusRTC_ver2 \
    && git submodule update --init --recursive \
    && mkdir -p build && cd build \
    && cmake .. -DBUILD_DOCUMENTATION=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && make -j$(nproc)


# Enter the container with a login shell
CMD ["bash", "-l"]
