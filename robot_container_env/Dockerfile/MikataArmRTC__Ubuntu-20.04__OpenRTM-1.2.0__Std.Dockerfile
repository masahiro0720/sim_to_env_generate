# Base image
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace

# OpenRTM Repo & APT
RUN rm -f /etc/apt/sources.list.d/openrtm.list \
    && echo "deb [trusted=yes] http://openrtm.org/pub/Linux/ubuntu focal main" > /etc/apt/sources.list.d/openrtm.list

# Install packages
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    cmake \
    doxygen \
    git \
    libboost-all-dev \
    libeigen-stl-dev \
    libomniorb4-dev \
    omniidl \
    omniorb-nameserver \
    openrtm-aist \
    python-is-python3 \
    python3-dev \
    python3-omniorb \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*








# Clone and Build C++ RTC: MikataArmRTC
RUN git clone -b main https://github.com/ogata-lab-admin/MikataArmRTC.git \
    && cd MikataArmRTC \
    && git submodule update --init --recursive \
    && mkdir -p build && cd build \
    && cmake .. -DBUILD_DOCUMENTATION=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && make -j$(nproc) && make install


CMD ["bash", "-l"]
