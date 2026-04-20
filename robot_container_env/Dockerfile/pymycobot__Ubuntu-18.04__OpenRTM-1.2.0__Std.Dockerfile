# Base image
FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace



# OpenRTM Repo & APT
RUN rm -f /etc/apt/sources.list.d/openrtm.list \
    && echo "deb [trusted=yes] http://openrtm.org/pub/Linux/ubuntu bionic main" > /etc/apt/sources.list.d/openrtm.list

# Install packages
# Added --allow-unauthenticated for safety on older repos
RUN apt-get update \
    && apt-get install -y --allow-unauthenticated \
    build-essential \
    cmake \
    doxygen \
    git \
    libboost-all-dev \
    omniidl \
    omniorb-nameserver \
    openrtm-aist \
    python3-dev \
    python3-omniorb \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*




# Upgrade pip and Fix setuptools version for older Python compatibility
RUN pip3 install --upgrade pip && pip3 install "setuptools<60.0.0" wheel



# Clone and Install Python Component: pymycobot
RUN git clone -b main https://github.com/elephantrobotics/pymycobot.git \
    && cd pymycobot \
    && if [ -f requirements.txt ]; then pip3 install -r requirements.txt; fi \
    && if [ -f setup.py ]; then python3 setup.py install; fi


CMD ["bash", "-l"]
