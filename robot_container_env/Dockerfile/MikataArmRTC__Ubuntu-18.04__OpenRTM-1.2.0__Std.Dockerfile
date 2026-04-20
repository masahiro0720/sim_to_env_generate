# Base image (Pinned by SHA256 Digest for Reproducibility)
FROM docker.io/library/ubuntu:18.04@sha256:152dc042452c496007f07ca9127571cb9c29697f42acbfad72324b2bb2e43c98

# --- OCI Multi-Architecture Support ---
ARG TARGETARCH
LABEL org.opencontainers.image.architecture="${TARGETARCH:-amd64}"
# OCI Annotations (Image Metadata)
LABEL org.opencontainers.image.authors="Ogata Labratory"
LABEL org.opencontainers.image.description="「Mikata Arm」というロボットアームを制御するためのソフトウェアモジュールである"

# --------------------------------------

ENV DEBIAN_FRONTEND=noninteractive

# Install sudo first
RUN apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/*



# OpenRTM Repo & APT
RUN rm -f /etc/apt/sources.list.d/openrtm.list \
    && echo "deb [trusted=yes] http://openrtm.org/pub/Linux/ubuntu bionic main" > /etc/apt/sources.list.d/openrtm.list

# Install packages
RUN apt-get update \
    && apt-get install -y --allow-unauthenticated \
    build-essential \
    cmake \
    doxygen \
    git \
    libboost-all-dev \
    libeigen3-dev \
    libomniorb4-dev \
    omniidl \
    omniorb-nameserver \
    openrtm-aist \
    openrtm-aist-dev \
    pkg-config \
    python3-dev \
    python3-omniorb \
    python3-pip \
    uuid-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*





# --- OCI Security: Create non-root user (rtmuser) ---
ARG USERNAME=rtmuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN mkdir -p /workspace/workspace && chown -R $USERNAME:$USERNAME /workspace
USER $USERNAME
WORKDIR /workspace/workspace
# ---------------------------------------------------


# Clone and Build C++ RTC: MikataArmRTC
RUN git clone -b master https://github.com/ogata-lab-admin/MikataArmRTC.git \
    && cd MikataArmRTC \
    && git submodule update --init --recursive \
    && mkdir -p build && cd build \
    && cmake .. -DBUILD_DOCUMENTATION=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && make -j$(nproc)


# --- OCI Execution Control: ENTRYPOINT ---
# Create entrypoint script
RUN echo '#!/bin/bash' > /workspace/entrypoint.sh && \
    echo 'set -e' >> /workspace/entrypoint.sh && \
    echo '# Add RTM specific environment variables here if needed in the future' >> /workspace/entrypoint.sh && \
    echo 'exec "$@"' >> /workspace/entrypoint.sh && \
    sudo chmod +x /workspace/entrypoint.sh

# RTM用のログディレクトリを作成
RUN mkdir -p /workspace/workspace/logs && sudo chown $USERNAME:$USERNAME /workspace/workspace/logs

WORKDIR /workspace/workspace

# --- OCI State Management: VOLUME ---
# ログファイル等のステートフルなデータをコンテナから分離
VOLUME ["/workspace/workspace/logs"]
# ------------------------------------

STOPSIGNAL SIGINT
ENTRYPOINT ["/workspace/entrypoint.sh"]
CMD ["bash"]
