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

## Environment and Build

Follow [Installation](../README.md#installation) for a fresh environment.
Use [Running Examples](running_examples.md) for the existing `idto_jazzy` container.
Record the checkout SHA, OS, CPU, RAM, compiler, Python, and Drake versions with each evaluation.
Keep Python bindings and `pydrake` consistent with the C++ Drake installation.

Inside the container:

```bash
cmake --build /root/idto_build -j 8
ctest --test-dir /root/idto_build --output-on-failure
```

The registered example tests use `--test`, which disables MPC and performs ten iterations.
Also run normal examples and inspect their closed-loop behavior.

## Run the examples

Run these sequentially, preserving logs and generated CSV files per example:

```bash
/root/idto_build/examples/pendulum/pendulum
/root/idto_build/examples/acrobot/acrobot
/root/idto_build/examples/spinner/spinner
/root/idto_build/examples/hopper/hopper
/root/idto_build/examples/jaco_ball/jaco_ball
/root/idto_build/examples/jaco/jaco
/root/idto_build/examples/dual_jaco/dual_jaco
/root/idto_build/examples/mini_cheetah/mini_cheetah
/root/idto_build/examples/allegro_hand/allegro_hand
/root/idto_build/examples/allegro_hand/allegro_hand --upside_down
/root/idto_build/examples/kuka/kuka
/root/idto_build/examples/punyo/punyo
/root/idto_build/examples/airhockey/airhockey
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
