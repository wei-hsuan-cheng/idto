# Inverse Dynamics Trajectory Optimization

Implements the contact-implicit trajectory optimization algorithm described in [Inverse Dynamics Trajectory Optimization for Contact-Implicit Model Predictive Control](https://idto.github.io/) by Vince Kurtz, Alejandro Castro, Aykut Özgün Önol, and Hai Lin.
See the [paper](https://arxiv.org/abs/2309.01813) for the formulation.

<img src="img/mini_cheetah.png" width="200"><img src="img/dual_jaco.png" width="200"><img src="img/allegro.png" width="200">

<img src="img/spinner.png" width="200"><img src="img/jaco_ball.png" width="200"><img src="img/hopper.png" width="200">

<img src="img/jaco.png" width="200"><img src="img/acrobot.png" width="200"><img src="img/punyo.png" width="200">


## Installation

The provided environment uses Ubuntu 24.04, ROS 2 Jazzy, and Drake 1.53.
IDTO itself uses Drake for dynamics and simulation; the examples do not require ROS launch or MuJoCo.

Use a Linux host or VM with Git and Docker Engine installed.
The following steps create the images, container, and build from scratch.
No pre-existing `ros_jazzy` or `idto_jazzy` image is required.


### 1. Get the Sources

On the Docker host, choose the directory to mount into the container:

```bash
export SRC_DIR="$HOME/src"
mkdir -p "$SRC_DIR"

git clone --branch jazzy \
  https://github.com/wei-hsuan-cheng/ros2_docker.git \
  "$SRC_DIR/ros2_docker"

git clone --branch eval/arm64 \
  https://github.com/wei-hsuan-cheng/idto.git \
  "$SRC_DIR/idto"
```

If the sources already exist, reuse their directories instead of cloning again.
For a shared source folder, set `SRC_DIR` to its path on the Docker host.


### 2. Build the Environment

The IDTO Dockerfile extends the `ros_jazzy` image.
Build the base image first, then the IDTO image:

```bash
docker build -t ros_jazzy "$SRC_DIR/ros2_docker"

docker build -t idto_jazzy \
  -f "$SRC_DIR/idto/docker/jazzy_arm64/Dockerfile" \
  "$SRC_DIR/idto"
```

The second image installs Drake, its prerequisites, and the additional build dependencies.
The first build requires network access.


### 3. Create the Container

Mount the source directory and keep compiled files in a persistent volume:

```bash
docker volume create idto_build

docker run -dit --name idto_jazzy \
  -p 7005:7000 \
  --mount type=bind,src="$SRC_DIR",dst=/root/src \
  --mount type=volume,src=idto_build,dst=/root/idto_build \
  idto_jazzy bash --noprofile --norc

docker exec -it idto_jazzy bash --noprofile --norc
```

This minimal container exposes Meshcat for browser visualization and does not need desktop display forwarding.
If `idto_jazzy` already exists, use `docker start idto_jazzy` and `docker exec` rather than creating another container.


### 4. Build IDTO

Inside the container:

```bash
cmake -S /root/src/idto \
  -B /root/idto_build -DCMAKE_BUILD_TYPE=Release
cmake --build /root/idto_build -j 8
ctest --test-dir /root/idto_build --output-on-failure
```

Do not source ROS before configuring or building.
The image already sets `CMAKE_PREFIX_PATH` and `PYTHONPATH` for Drake.
CMake builds the C++ examples and the `pyidto` Python bindings.
Keep `pydrake` and the C++ library from the same Drake installation.

The source mount reflects edits immediately.
Rebuild after changing source files or example YAMLs.
Compiled files remain in the volume when the container is recreated.


## Running Examples

See [Running Examples](docs/running_examples.md) for individual commands, Meshcat playback, batch runs, configuration, and recorded outputs.
For a container on the local machine, Meshcat is available at `http://localhost:7005` while an example runs.
For a remote Docker host, use that host's address instead.


## Documentation

- [Running examples](docs/running_examples.md)
- [Evaluation report](docs/evaluation_report.md)
- [Handoff](docs/handoff.md)
- [Evaluation plan](docs/evaluation_plan.md)
