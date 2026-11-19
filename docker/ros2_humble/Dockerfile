# ROS 2 Humble (Ubuntu 22.04, x86-64) with Drake 1.30.0: the configuration IDTO's README recommends, as a slim
# ros-base image. Drake has no ARM64 build for Ubuntu 22.04, so on an ARM64 host (a Parallels VM on Apple Silicon)
# this image runs under QEMU user emulation; register it once per host boot:
#
#   docker run --privileged --rm tonistiigi/binfmt --install amd64
#
# The source is not copied in: the container mounts ~/src and builds into /root/idto_build (docker-compose.yml).
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive TZ=America/Los_Angeles
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget vim ninja-build libgflags-dev python3-pip python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

# Drake 1.30.0 binary release and its runtime prerequisites (as IDTO's Dockerfile, at /opt/drake).
RUN wget -q https://github.com/RobotLocomotion/drake/releases/download/v1.30.0/drake-1.30.0-jammy.tar.gz \
    && tar -xzf drake-1.30.0-jammy.tar.gz -C /opt && rm drake-1.30.0-jammy.tar.gz
RUN apt-get update && /opt/drake/share/drake/setup/install_prereqs -y && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir scipy==1.12 pytest

# X11 and OpenGL clients, as the ros2_docker images. Under QEMU (ARM64 host) Mesa cannot drive the host GPU (virgl):
# QEMU user-mode does not translate the DRM ioctls, so OpenGL falls back to llvmpipe (software). IDTO itself needs no
# GPU: the optimizer and Drake's simulation are CPU-only, and Meshcat renders in the viewing browser.
RUN apt-get update && apt-get install -y --no-install-recommends mesa-utils x11-apps libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

ENV CMAKE_PREFIX_PATH=/opt/drake
ENV PYTHONPATH=/opt/drake/lib/python3.10/site-packages
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
WORKDIR /root
