FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG RUST_TOOLCHAIN=1.88.0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    clang \
    cmake \
    curl \
    git \
    libprotobuf-dev \
    libudev-dev \
    liburdfdom-tools \
    pkg-config \
    protobuf-compiler \
    python3-colcon-common-extensions \
    python3-pip \
    python3-protobuf \
    python3-pytest \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
    ros-jazzy-controller-manager \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-moveit \
    ros-jazzy-realtime-tools \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    mesa-utils \
    udev \
    usbutils \
    x11-utils \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --break-system-packages eclipse-zenoh==1.9.0

ENV RUSTUP_HOME=/opt/rustup \
    CARGO_HOME=/opt/cargo \
    PATH=/opt/cargo/bin:${PATH}

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
    | sh -s -- -y --profile minimal --default-toolchain "${RUST_TOOLCHAIN}" \
 && rustup component add --toolchain "${RUST_TOOLCHAIN}" rustfmt clippy \
 && rustc --version \
 && cargo --version

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo 'test ! -f /workspaces/hex_arm_ros2/install/setup.bash || source /workspaces/hex_arm_ros2/install/setup.bash' >> /root/.bashrc

WORKDIR /workspaces/hex_arm_ros2
CMD ["sleep", "infinity"]

