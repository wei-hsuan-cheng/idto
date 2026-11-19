# Native ARM64: ROS 2 Jazzy (Ubuntu 24.04) with Drake 1.53.0, the first release with Linux ARM64 binaries (IDTO's
# README pins 1.30.0, which is x86-64 only on Linux; docker/ros2_humble runs that under QEMU).
#
# The base defaults to ros_jazzy, the image of github.com/wei-hsuan-cheng/ros2_docker (jazzy branch): it already has
# the toolchain and Mesa pinned for a virgl GPU (Parallels), and reusing its layers keeps the disk cost to Drake.
# Elsewhere: --build-arg BASE=ros:jazzy-ros-base.
ARG BASE=ros_jazzy
FROM ${BASE}

ENV DEBIAN_FRONTEND=noninteractive
RUN wget -q https://github.com/RobotLocomotion/drake/releases/download/v1.53.0/drake-1.53.0-noble-arm64.tar.gz \
    && tar -xzf drake-1.53.0-noble-arm64.tar.gz -C /opt && rm drake-1.53.0-noble-arm64.tar.gz
RUN apt-get update && /opt/drake/share/drake/setup/install_prereqs -y \
    && apt-get install -y --no-install-recommends libgflags-dev ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Build IDTO without sourcing ROS (`bash -c`, not a login shell) so CMake finds Drake's dependencies, not ROS's.
ENV CMAKE_PREFIX_PATH=/opt/drake
ENV PYTHONPATH=/opt/drake/lib/python3.12/site-packages
WORKDIR /root
