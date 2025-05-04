# Use KasmVNC core image for Ubuntu 24.04 Noble
# Consider pinning to a specific version like :1.15.0 for stability
FROM ghcr.io/linuxserver/baseimage-kasmvnc:ubuntunoble

# Set Non-Interactive Frontend for apt-get
ENV DEBIAN_FRONTEND=noninteractive
# Kasm images often perform actions as root during build, then run the session as kasm-user (UID 1000)
USER root

# --- Basic Setup & Locales (Some might be redundant from base, but safe) ---
# Install essential tools and dependencies needed for ROS/Rust
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    build-essential \
    pkg-config \
    libssl-dev \
    libclang-dev \
    xterm 

RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# --- Install ROS 2 Jazzy Jalisco ---
# Add ROS 2 repo and install desktop variant + dev tools
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-dev-tools \
 && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup script automatically for all interactive bash sessions
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# --- Install Rust ---
# Install Rust system-wide using rustup
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable --profile default --no-modify-path

# Source cargo env automatically for all interactive bash sessions
RUN echo "source $CARGO_HOME/env" >> /etc/bash.bashrc

SHELL ["/bin/bash", "-c"]

# --- Add kasm-user to sudoers (optional, for convenience within the desktop session) ---
# The default user in Kasm sessions is 'kasm-user' (UID/GID 1000)
# This allows running sudo commands inside the container without a password prompt
RUN echo "kasm-user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/kasm-user && \
    chmod 0440 /etc/sudoers.d/kasm-user

# Add workspace sourcing to the kasm-user's bashrc for interactive shells
RUN echo "source /home/kasm-user/ros2_ws/install/setup.bash" >> /home/kasm-user/.bashrc && \
    chown 1000:1000 /home/kasm-user/.bashrc

# Add workspace sourcing to the kasm-user's bashrc for interactive shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/kasm-user/.bashrc && \
    chown 1000:1000 /home/kasm-user/.bashrc

# Set working directory
WORKDIR /usr/local/src

# Clone the photoneo repository
RUN git clone https://github.com/sequenceplanner/r2r_ur_controller.git

RUN . /opt/ros/jazzy/setup.bash

# Change to the correct path for the Rust project INSIDE the ALREADY CLONED repo
WORKDIR /usr/local/src/r2r_ur_controller/
# Build the Rust application
RUN source /opt/ros/jazzy/setup.bash && \
    echo "--- Building Rust crate ---" && \
    cargo build --release

# Reset WORKDIR back to root
WORKDIR /

# Start bash script to launch everything
COPY start.sh /start.sh
RUN chmod +x /start.sh

CMD ["/start.sh"]
