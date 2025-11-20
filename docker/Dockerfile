# Base Image
FROM ubuntu:24.04

# Set non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Core Dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    ninja-build \
    gperf \
    python3-pip \
    python3-venv \
    git \
    make \
    gcc \
    g++ \
    libsdl2-dev \
    # Micro-ROS Specifics
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-lark-parser \
    # Flashing
    openocd \
    usbutils \
    # Requested Tools
    neovim \
    ack-grep \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Zephyr SDK
RUN wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.1/zephyr-sdk-0.16.1_linux-x86_64.tar.gz && \
    tar -xzf zephyr-sdk-0.16.1_linux-x86_64.tar.gz -C /opt && \
    rm zephyr-sdk-0.16.1_linux-x86_64.tar.gz && \
    /opt/zephyr-sdk-0.16.1/setup.sh -t arm-zephyr-eabi

# Create a non-root user
RUN useradd -m -s /bin/bash -G sudo dev && \
    echo "dev:dev" | chpasswd && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set up west tool
USER dev
WORKDIR /home/dev
RUN pip3 install --user west
ENV PATH="/home/dev/.local/bin:${PATH}"

# Set Zephyr environment variables
ENV ZEPHYR_BASE=/home/dev/workspace/zephyr
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-0.16.1

# Default command
CMD ["/bin/bash"]
