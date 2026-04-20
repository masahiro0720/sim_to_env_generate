# Base image
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace



# OpenRTM official repository skipped for newer Ubuntu (Using OS standard packages)

# Install packages
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    cmake \
    doxygen \
    git \
    libboost-all-dev \
    omniidl \
    omniorb-nameserver \
    openrtm-aist \
    python-is-python3 \
    python3-dev \
    python3-omniorb \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*




# Upgrade pip
RUN pip3 install --upgrade pip setuptools wheel



# Clone and Install Python Component: pymycobot
RUN git clone -b main https://github.com/elephantrobotics/pymycobot.git \
    && cd pymycobot \
    && if [ -f requirements.txt ]; then pip3 install -r requirements.txt; fi \
    && if [ -f setup.py ]; then python3 setup.py install; fi


CMD ["bash", "-l"]
