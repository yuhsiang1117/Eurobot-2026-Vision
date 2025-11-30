FROM ros:humble AS base

# 1. Install Node.js v20 (Required for Gemini)
RUN apt-get update && apt-get install -y curl gnupg && \
    curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs

# 2. Install Gemini CLI Globaly
RUN npm install -g @google/gemini-cli

# Keep apt cache to speed up subsequent builds
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Install common tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    curl \
    git \
    sudo \
    usbutils \
    v4l-utils \
    htop \
    iputils-ping \
    wget \
    net-tools \
    tmux \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Python pip
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
        ros-humble-ros-base \
        ros-humble-image-transport-plugins \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-pcl-ros \
        ros-humble-octomap-ros \
        ros-humble-octomap-msgs \
        ros-humble-rviz2 \
        ros-humble-tf2-geometry-msgs \
        ros-humble-tf2-eigen \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=vision
ARG USER_UID=1000

RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash -u $USER_UID -G sudo $USERNAME \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY --chown=$USERNAME:$USERNAME \
     modules/install_realsense.sh /tmp/install_realsense.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_realsense.sh && rm /tmp/install_realsense.sh

ENTRYPOINT []
CMD ["/bin/bash"]