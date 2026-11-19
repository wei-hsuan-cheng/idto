Clone, build, and test IDTO, the Drake-based contact-implicit inverse-dynamics trajectory optimizer.

Repository
https://github.com/ToyotaResearchInstitute/idto

Paper
https://arxiv.org/html/2309.01813v3

Project
https://idto.github.io/

## Goal

Reproduce the existing examples and assess whether IDTO is a suitable starting
point for multi-contact, surface-aware whole-body inverse-dynamics MPC.

I particularly care about contacts moving along spherical surfaces and multiple
spheres approximating robot links or objects.

## Environment and build

1. Read applicable AGENTS.md instructions and the repository README.
2. Record the checkout SHA, OS, CPU, RAM, compiler, Python, and Drake versions.
3. Follow the repository's supported installation instructions. The inspected
   README recommends Ubuntu 22.04 and Drake 1.30.0.
4. Prefer the provided Docker setup when appropriate. Diagnose recoverable
   problems and document any required changes.
5. Keep Python bindings and pydrake consistent with the Drake C++ installation.

Starting commands:

```bash
git clone https://github.com/ToyotaResearchInstitute/idto.git
cd idto
git rev-parse HEAD
docker build -t idto .
docker run --name idto-test -it \
  -p 127.0.0.1:7000:7000 \
  idto
```

For a compatible native Drake installation:

```bash
cmake -S . -B build
cmake --build build -j 4
```

## Run registered tests

```bash
ctest --test-dir build -N
ctest --test-dir build --output-on-failure
```

Run existing Python tests if the bindings are installed.

Inspect what each test actually checks. In the shared example runner, --test
disables MPC and performs a shortened optimization, so also run normal examples.

## Run the examples

Run these sequentially, preserving logs and generated CSV files per example:

```bash
./build/examples/pendulum/pendulum
./build/examples/acrobot/acrobot
./build/examples/spinner/spinner
./build/examples/hopper/hopper
./build/examples/jaco_ball/jaco_ball
./build/examples/jaco/jaco
./build/examples/dual_jaco/dual_jaco
./build/examples/mini_cheetah/mini_cheetah
./build/examples/allegro_hand/allegro_hand
./build/examples/allegro_hand/allegro_hand --upside_down
./build/examples/kuka/kuka
./build/examples/punyo/punyo
./build/examples/airhockey/airhockey
```

Verify paths and supported flags against the checkout.

Inspect the Meshcat visualization when available. Record whether each example
performs offline optimization, trajectory playback, or closed-loop simulation.
Use practical timeouts and continue with other examples after documenting a failure.

## Understand the implementation

Trace these files:

- examples/<name>/<name>.cc
- examples/<name>/<name>.yaml
- examples/example_base.cc
- examples/mpc_controller.cc
- examples/yaml_config.h
- optimizer/trajectory_optimizer.cc
- optimizer/solver_parameters.h
- utils/find_resource.*

Explain how Drake supplies geometry queries and rigid-body dynamics, while IDTO
supplies smooth contact forces and trajectory optimization.

Identify the separate planning and simulation contact models.

Check how source YAMLs are copied into the build tree. After changing source
configuration, rerun CMake and verify which file is actually loaded.

## Validate moving-contact gradients

For jaco_ball or Allegro:

1. Select a nondegenerate sphere-contact configuration.
2. Record contact pair IDs, body-local/world contact points, normals, and gaps.
3. Perturb a joint configuration and recompute geometry and inverse dynamics.
4. Verify that the contact point moves along the body surface.
5. Compare the configured ID derivative with an independent central difference
   over several perturbation sizes.
6. Explain how changing contact location affects J_c(q)^T f_c(q,v).

Use valid configuration perturbations for floating bases and unit quaternions.
Distinguish geometric closest-point motion from contacting material-point velocity.

Start with the example's derivative method. Investigate other available methods
only after obtaining a working baseline.

## Measure real-time capability

Separate initial trajectory optimization from steady-state MPC updates.

For spinner, dual_jaco, mini_cheetah, and allegro_hand, measure:

- Per-update solve latency: median, p95, p99, maximum, and sample count.
- Complete controller-update latency if different.
- Deadline misses relative to controller_frequency.
- Horizon, timestep, optimizer iterations, and thread count.
- Simulated time divided by wall-clock time.
- Task tracking and contact behavior.

Inspect SolveFromWarmStart() in examples/mpc_controller.cc. Add minimal timing
instrumentation if necessary and preserve its diff.

Verify what solver_stats.csv measures before using it as MPC timing evidence.
A configured controller frequency alone does not establish real-time performance.

## Deliverables

Produce a Markdown report containing:

1. Environment, checkout SHA, and reproducible build/run commands.
2. Registered-test results.
3. A row for every attempted example with effective mode, exit/solver status,
   observed task outcome, runtime, and log locations.
4. MPC latency statistics and deadline misses.
5. Moving-contact gradient verification.
6. Required fixes and configuration changes.
7. Remaining blockers and assessment for custom multi-sphere WB-ID MPC.

Complete the existing-example baseline before implementing a custom robot.
