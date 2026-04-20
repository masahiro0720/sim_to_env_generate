# Base image
FROM docker.io/library/ubuntu:16.04
# OCI Annotations (Image Metadata)
LABEL org.opencontainers.image.description="「Mikata Arm」というロボットアームを制御するためのソフトウェアモジュールである"

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace


# Fix for EOL (End Of Life) Ubuntu versions
# Switch repositories to old-releases.ubuntu.com
RUN sed -i -r 's/([a-z]{2}\.)?archive.ubuntu.com/old-releases.ubuntu.com/g' /etc/apt/sources.list \
    && sed -i -r 's/security.ubuntu.com/old-releases.ubuntu.com/g' /etc/apt/sources.list


# OpenRTM Repo & APT
RUN rm -f /etc/apt/sources.list.d/openrtm.list \
    && echo "deb [trusted=yes] http://openrtm.org/pub/Linux/ubuntu xenial main" > /etc/apt/sources.list.d/openrtm.list

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
    python3-dev \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*







CMD ["bash", "-l"]
