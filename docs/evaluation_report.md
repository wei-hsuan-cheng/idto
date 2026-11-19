# IDTO evaluation: build, examples, moving-contact gradients, MPC timing

Evaluation of [IDTO](https://github.com/ToyotaResearchInstitute/idto) against [the plan](evaluation_plan.md): reproduce
the examples and judge IDTO as a starting point for multi-contact, surface-aware whole-body inverse-dynamics MPC, with
contacts moving along spheres. Run 2026-11-19.

Logs, CSVs and Meshcat recordings are in `~/src/mpc/contact_implicit_mpc/idto_runs/` on the Mac
(`/root/src/mpc/contact_implicit_mpc/idto_runs/` in the container); paths below are relative to it.

**Summary**

- Builds unmodified on IDTO's recommended setup (Ubuntu 22.04, Drake 1.30.0). All 23 registered tests pass, and all
  13 example runs exit cleanly and reach their task.
- Moving sphere contacts behave correctly: the witness points slide along both spheres exactly as the closed form
  predicts, and the configured derivatives match central differences to about 2e-7 for joints and ball position.
- One systematic error: every gradient method drops the dN+/dq term, so derivatives along a rotating floating base
  are 0.1-6.6 % off.
- Real time: yes, natively. Under x86 emulation every MPC update misses its deadline. Ported to Drake 1.53 (four small
  changes, still building against 1.30) and run natively on ARM64, every MPC example meets its configured rate with
  2.6-7.9x headroom, for example spinner 549 Hz against 200 and mini_cheetah 242 Hz against 60 (section 8).
- On Drake 1.53 the simulated plant changes: the new default contact solver (SAP) drops Allegro's ball. The planner
  itself is unchanged (section 8).

## 1. Environment and reproduction

| item | value |
|---|---|
| IDTO | fork `wei-hsuan-cheng/idto`, upstream `de0629c7811aa9b330e56c4385629005b09495f0` (+ `39c5022`, the plan) |
| host | Parallels VM on an Apple Silicon Mac: Ubuntu 24.04.3, **aarch64**, 8 vCPUs, 7.5 GB RAM, Docker 29.2.1 |
| container | `idto_humble`: `ros:humble-ros-base` (Ubuntu 22.04.5), **linux/amd64 under QEMU user emulation** |
| compiler / build | GCC 11.4.0, CMake 3.22.1, `-O3`, Release |
| Python | 3.10.12, numpy 1.26.4, scipy 1.12.0 |
| Drake | 1.30.0 binary (`66be6c1eb272f5bb880134fc0e933d1269ea632b`), C++ and pydrake from the same `/opt/drake` |
| AGENTS.md | none in the repository; the README is the only instruction |

**Why x86 emulation.**
- Drake 1.30.0 ships only x86-64 Linux binaries (`jammy`/`noble` tarballs, amd64 debs).
- Native Linux ARM64 Drake binaries start at v1.53, and only for Ubuntu 24.04 and newer.
- The README's configuration (Ubuntu 22.04, Drake 1.30.0) therefore runs on this ARM64 VM only as an emulated x86
  container.

Functionally this is exact. Timing is several times slower than native (section 4).

**No GPU in the x86 container.**
- The container has the same display and GPU wiring as `ros2_docker`'s `ros_jazzy`: privileged, host IPC,
  `/dev/dri`, X11 on `:1`.
- Mesa inside still renders with `llvmpipe` (software), while the native ARM64 `ros_jazzy` gets
  `virgl (Apple M2 Pro)`. QEMU user-mode emulation does not translate the virtio-GPU DRM ioctls ("failed to get
  driver name", "failed to get magic").
- IDTO does not need a GPU: the optimizer and Drake's simulation are CPU-only, and Meshcat's 3D view renders in the
  viewing browser, on the Mac's GPU.

```bash
# once per VM boot: register x86-64 emulation (no sudo needed)
docker run --privileged --rm tonistiigi/binfmt --install amd64
# image and container (docker/ros2_humble/; mounts the Mac's ~/src at /root/src, publishes Meshcat 7000-7004)
cd /media/psf/Home/src/mpc/contact_implicit_mpc/idto
docker compose -f docker/ros2_humble/docker-compose.yml build      # about 15 min emulated
docker compose -f docker/ros2_humble/docker-compose.yml up -d
# build out of tree inside the container (12.5 min emulated, -j4)
docker exec idto_humble bash -lc "cmake -S /root/src/mpc/contact_implicit_mpc/idto -B /root/idto_build && cmake --build /root/idto_build -j 4"
# tests, examples, gradient check
docker exec idto_humble bash -lc "ctest --test-dir /root/idto_build --output-on-failure"
docker exec idto_humble bash -lc "bash /root/src/mpc/contact_implicit_mpc/idto_runs/run_examples.sh 3600"
docker exec idto_humble bash -lc "/root/idto_build/examples/allegro_hand/contact_gradient_check --ball_spin 2 --gradients_method forward_differences"
```

**Meshcat.**
- Live while an example runs, at `http://10.211.55.17:7000` from the Mac (checked: HTTP 200).
- The MPC runs of spinner, dual_jaco, mini_cheetah and allegro_hand are also saved as static HTML recordings:
  `examples/<name>/meshcat_recording.html`, which open in any browser.

## 2. Registered tests

`ctest -N`: 23 tests. All pass, 105 s in total (`tests/ctest_run.log`).

| group | tests | what they check |
|---|---|---|
| Python bindings | `problem_definition`, `solver_parameters`, `trajectory_optimizer_solution`, `trajectory_optimizer_stats`, `find_resource`, `trajectory_optimizer`, `warm_start` (`*_test_py`) | the `pyidto` module built by CMake in the build tree (linked to the same `/opt/drake`) constructs, solves and warm-starts |
| C++ unit | `profiler_test`, `find_resource_test`, `penta_diagonal_solver_test`, `trajectory_optimizer_test` | pentadiagonal matrix operations and solves; the optimizer's velocities, inverse dynamics, cost, gradients against finite differences and autodiff (pendulum, kuka), `ContactGradientMethods` (the derivative methods agree with each other), `QuaternionDofs`, Hessians, dogleg and trust ratio, equality constraints (spinner, hopper), a pendulum swing-up |
| examples | `<name>_test` for the 12 examples (`<name> --test`) | smoke test only (see below) |

The example tests check almost nothing about behaviour. `--test` forces:
- `mpc = false`, `max_iters = 10`, `num_threads = 1`;
- no playback and no `solver_stats.csv`.

So a pass means only that the problem builds and ten iterations run without throwing. Behaviour was checked by running
the examples themselves.

`ContactGradientMethods` cannot catch the floating-base derivative bias in section 5, because every method it compares
holds N+ fixed in the same way.

## 3. Examples

The YAMLs are copied into the build tree when CMake configures (`configure_file(... COPYONLY)`), and
`FindIdtoResource` resolves `idto/...` against `IDTO_BINARY_DIR`, the build directory compiled into the binary. A
binary therefore always reads the build-tree copy.

This was verified:
- The build-tree `spinner.yaml` lacked a key just added to the source.
- A plain `cmake --build` re-ran configure (the source YAML is a configure dependency), and every copy then had it.

**After editing a source YAML, rebuild before running.** Building only a target that did not exist before fails
first: for example `--target contact_gradient_check` returned "No rule to make target". Run `cmake -S ... -B ...`
in that case.

All 13 runs exit 0 (`examples_summary*.log`, `examples/<name>/{run.log,status.txt}`). "Initial solve" is the full
optimization that also warm-starts MPC. Outcomes are measured on the simulated state (`examples/<name>/sim_state.csv`,
100 Hz) or, for open-loop examples, on the plan.

| example | effective mode | initial solve | outcome | wall time |
|---|---|---|---|---|
| pendulum | closed-loop MPC sim, 5 s | max iterations, 0.59 s | swings up; upright from 2.4 s, error < 0.001 rad in the last 1 s | 30 s |
| acrobot | closed-loop MPC sim, 5 s | max iterations, 1.13 s | swings up to [3.140, 0.008] (target [pi, 0]) | 28 s |
| spinner | closed-loop MPC sim, 5 s | max iterations, 2.76 s | flicks the spinner continuously: 4.59 rad in 5 s, about 1.0 rad/s | 30 s |
| hopper | closed-loop MPC sim, 5 s | max iterations, 8.39 s | hops towards the goal: x 0 -> -0.375 m (target -0.5) | 38 s |
| jaco_ball | closed-loop MPC sim, 5 s | max iterations, 1.56 s | rolls the ball y 0.27 -> 0.079 m (target 0), the ball turns 179 deg | 46 s |
| jaco | closed-loop MPC sim, 5 s | max iterations, 13.9 s | box orientation error 90 -> 6.9 deg, box at (0.478, -0.116) m (target (0.5, -0.1)) | 59 s |
| dual_jaco | closed-loop MPC sim, 5 s | max iterations, 5.24 s | box turned 46 -> 4.2 deg error and lifted 0.08 -> 0.105 m (target 0.1) | 52 s |
| mini_cheetah | closed-loop MPC sim, 10 s | max iterations, 4.33 s | trots 3.51 m forward (0.35 m/s, target 0.4 m/s), height 0.29-0.35 m, max tilt 8.8 deg | 77 s |
| allegro_hand | closed-loop MPC sim, 10 s | max iterations, 21.7 s | ball orientation error 90 -> 12.2 deg (min 8.7), ball drifts at most 16 mm | 177 s |
| allegro_hand `--upside_down` | **open loop** (single optimization, plan played back) | max iterations, 25.5 s | plan ends at 12.7 deg ball orientation error, ball within 3 mm | 32 s |
| kuka | **open loop**, plan played back | max iterations, 45.6 s | plan pushes the box 0.8 m to within 1.1 cm, 2.7 deg | 50 s |
| punyo | closed-loop MPC sim, 10 s | max iterations, 26.1 s | lifts the ball 0.2 -> 0.79 m (target 0.8), 3 cm from target | 125 s |
| airhockey | **open loop**, plan played back | max iterations, 61.4 s | plan puts the puck at (0.317, 1.024) (target (0.3, 1.0)); puck angle 0.11 rad (target 1.0) | 70 s |

**How the closed loop is built (`example_base.cc`).** The simulated outcomes are valid despite the slow emulation.

- The MPC is a Drake `LeafSystem` whose periodic unrestricted update (every `1 / controller_frequency`) calls
  `SolveFromWarmStart` with `max_iterations = mpc_iters` (1 or 2).
- A `DiscreteTimeDelay` of exactly one replan period sits between the controller and the interpolator / PD+ servo,
  which models one period of solver latency.
- The `Simulator` is synchronous: it waits for each solve. Simulated time therefore never sees the real solve time;
  the simulation shows what happens *if* every solve meets its deadline.
- `sim_realtime_rate` (0 or 1 in the YAMLs) only throttles the simulation when it runs faster than real time.

**Two contact models.**

- **Planning model** (`CreatePlantModel`): IDTO's own smooth compliant contact in
  `TrajectoryOptimizer::CalcContactForceContribution`. For every pair returned by Drake's
  `ComputeSignedDistancePairwiseClosestPoints(threshold)`:
  - the contact point is the midpoint of the two witness points;
  - the normal force is `sigma k log(1 + exp(-phi / sigma))`, times a Hunt-Crossley-like dissipation factor in the
    normal slip `vn / dissipation_velocity`;
  - friction is regularized Coulomb, `-mu fn vt / sqrt(vs^2 + |vt|^2)`.

  The forces enter as body spatial forces through `plant().CalcInverseDynamics(context, a, f_ext)`: Drake supplies
  geometry, kinematics and rigid-body dynamics, IDTO supplies the contact force law and the optimizer.
- **Simulation model** (`CreatePlantModelForSimulation`): Drake's discrete `MultibodyPlant` at `sim_time_step` with
  Drake 1.30's defaults: the TAMSI solver and `hydroelastic_with_fallback` contact. The Allegro hand spheres and the
  simulated ball carry compliant hydroelastic properties; the simulation also adds a ground box.
- The planner is MPC against a deliberately different model. For example, Allegro plans with k = 100 N/m and
  sigma = 1 mm against a hydroelastic simulation.

## 4. MPC latency and deadline misses

The timing instrumentation is preserved in `instrumentation.diff`, about 75 added lines. It records, per MPC update,
the wall-clock time of `SolveFromWarmStart` and of the whole update, the optimizer's own `solve_time`, and the
iteration count (`examples/<name>/mpc_timing.csv`). It also prints the closed loop's simulated-to-wall time ratio.

`solver_stats.csv` is not MPC timing evidence: it is written once, for the initial full solve
(`SolveTrajectoryOptimization`), and never for MPC updates.

The table below is **emulated x86 on an ARM64 VM**. It shows that the loop works and how the cost scales between
examples, but not native real-time capability. The deadline is the replan period. Threads: `num_threads: 4` for the
OpenMP derivative loops (pendulum 1). Horizon: `num_steps` x `time_step` = 40 x 0.05 s, except jaco_ball 10,
dual_jaco and mini_cheetah 20.

| example | updates | period [ms] | solve median / p95 / p99 / max [ms] | update median / max [ms] | misses | iters | sim/wall |
|---|---|---|---|---|---|---|---|
| pendulum | 1500 | 3.3 | 14.7 / 16.1 / 19.2 / 93.3 | 16.7 / 99.0 | 100 % | 1 | 0.19 |
| acrobot | 1500 | 3.3 | 13.4 / 14.5 / 18.3 / 101.6 | 16.1 / 121.4 | 100 % | 1 | 0.19 |
| **spinner** | 1000 | 5.0 | 18.7 / 24.3 / 33.6 / 89.2 | 22.9 / 122.2 | 100 % | 1 | 0.19 |
| hopper | 500 | 10.0 | 45.4 / 52.6 / 60.0 / 97.2 | 51.3 / 103.6 | 100 % | 1 | 0.18 |
| jaco_ball | 500 | 10.0 | 74.1 / 79.9 / 85.0 / 106.4 | 79.0 / 111.6 | 100 % | 2 | 0.12 |
| jaco | 100 | 50.0 | 297.3 / 312.9 / 321.2 / 518.7 | 312.3 / 534.5 | 100 % | 1 | 0.13 |
| **dual_jaco** | 100 | 50.0 | 254.8 / 263.6 / 274.7 / 284.6 | 267.9 / 297.8 | 100 % | 2 | 0.13 |
| **mini_cheetah** | 600 | 16.7 | 84.3 / 88.5 / 109.9 / 366.6 | 96.1 / 394.7 | 100 % | 1 | 0.14 |
| **allegro_hand** | 100 | 100.0 | 627.4 / 655.0 / 830.1 / 849.9 | 653.6 / 876.9 | 100 % | 1 | 0.07 |
| punyo | 150 | 66.7 | 544.6 / 596.7 / 691.7 / 738.0 | 568.3 / 762.0 | 100 % | 1 | 0.11 |

- **Solve time vs. update time.** The full update adds 2-26 ms over the solve: re-interpolating the stored trajectory
  as a warm start, `std::cout`, and storing the solution as cubic splines.
- **The optimizer's own `solve_time`** is 83-99 % of the wall-clock solve.
- **What `SolveFromWarmStart` does.** It runs a fixed number of trust-region (dogleg) iterations from the shifted
  previous plan, `q_guess[0]` = the measured `q0`. It never reports convergence inside MPC, because `max_iterations` is
  1 or 2.
- **Estimating native speed.** The emulated rate is about 1/4 to 1/8 of each example's configured
  `controller_frequency`. A native rate cannot be inferred from that; it needs a native build (section 7).

## 5. Moving-contact gradient verification (Allegro, sphere-sphere)

The program is `examples/allegro_hand/contact_gradient_check.cc` (new target `contact_gradient_check`). It uses the
example's planning model and contact parameters (k 100 N/m, sigma 1 mm, mu 1, stiction 0.1 m/s), evaluates
`tau[1] = ID(q[2], v[2], a[1])` through IDTO's own `TrajectoryOptimizer`, and compares it with IDTO's partials
`dtau_dqp[1] = d tau[1] / d q[2]`. Logs are in `gradient/`, one per method x ball spin.

**1. Configuration.**
- Contact: the closest finger-ball pair at `q_init` is `allegro_hand_left::link_11_tip_collision_1` (r 15 mm) |
  `ball::ball_collision` (r 60 mm). Its finger joint `q[0]` was pressed until phi = -1.000 mm. Geometry names are
  scoped by model instance (`ball::ball_collision`), so the tool matches pairs by geometry id.
- Motion: joint `q[1]` slides that contact along the ball fastest (0.057 m/rad). The trajectory moves `q[1]` by
  0.02 rad from `q[1]` to `q[2]`, so the finger slides.
- Pairs: 13 pairs lie within the force threshold (15.7 mm). The table in each log lists:
  - pair ids and scoped names, bodies, phi;
  - the normal `n_W`, the contact point `p_WC`;
  - the witness points in their geometry frames (`p_GaCa`, `p_GbCb`), the material-point slip `v_AcBc` and the force.

  For the chosen pair: `n_W` = (0.495, -0.851, -0.175), `p_GbCb` = (-0.0297, 0.0511, 0.0105) m (on the ball, |p| = r),
  `f_BC_W` = (0.112, -0.114, -0.034) N.

**2. The contact point moves along both surfaces.** Perturbing `q[1]` by -0.02 to +0.02 rad:
- the ball-side witness point stays on the ball (|p| - r ≤ 7e-18 m) and travels 0.57 mm along the surface per
  0.01 rad;
- the finger-side point stays on the finger sphere (≤ 5e-18 m);
- both equal the closed-form sphere-sphere closest points, c_ball + r (c_f - c_ball) / |c_f - c_ball|, to 0;
- phi changes smoothly from +0.05 mm to -2.03 mm.

**3. Configured derivative vs. independent central differences.**

The reference is central differences of `tau[1]` over h = 1e-2 ... 1e-7, recomputing geometry, v = N+(q) (q[2] - q[1]) / dt
and inverse dynamics. The figures are relative errors at the best h.

Ball orientation is perturbed only along valid unit-quaternion curves q (x) exp(h w / 2), compared with
D_IDTO · dq/dh, so the quaternion is never denormalized.

| direction | forward differences (as configured), ball static | ball spinning at 2 rad/s: forward differences | central differences | autodiff |
|---|---|---|---|---|
| sliding joint `q[1]` | 2.4e-7 | 2.1e-7 | 3.9e-9 | 6e-11 |
| pressing joint `q[0]` | 2.4e-7 | 2.2e-7 | 5.5e-9 | 8e-11 |
| all 16 hand joints (matrix) | 2.2e-7 | 2.0e-7 | 5.8e-8 | 6.8e-8 |
| ball position x / y / z | 4e-7 / 7e-7 / 3e-7 | 5e-7 / 5e-7 / 1e-6 | 6e-7 / 5e-7 / 3e-7 | 7e-7 / 6e-7 / 3e-7 |
| ball rotation x / y / z | 2e-8 / 4e-9 / 2e-8 | **1.25e-3 / 6.6e-2 / 3.8e-2** | **same** | **same** |

The error falls as O(h²) down to the noise floor, so the reference is sound.

- **Static ball.** With the ball at rest, all of IDTO's derivatives are consistent with the moving-contact geometry.
- **Spinning ball.** With the ball rotating, all three methods are 0.1-6.6 % off along rotations, and the error does
  not depend on h. The cause: IDTO computes v_t = N+(q_t) (q_t - q_{t-1}) / dt but differentiates with N+ held fixed:
  - forward and central differences perturb q, v and a with the nominal `Nplus[t]` column;
  - the autodiff path gives the identical numbers.

  The missing term is (dN+/dq) (q_t - q_{t-1}) / dt. It vanishes when the floating base does not rotate between knots,
  and it is a systematic bias for any rotating floating base: the ball here, a humanoid or quadruped base, a
  manipulated object.

**4. How the moving contact location enters J_c(q)^T f_c(q, v).** IDTO never forms J_c explicitly. It applies
F = [0; f] at C to body B (and -F to A), shifted to the body origins, and Drake's `CalcInverseDynamics` maps body
forces to generalized forces, i.e. J_{B,C}(q)^T f - J_{A,C}(q)^T f.

- **Replica.** Rebuilding that from the pairs gives IDTO's inverse dynamics to 2.5e-16.
- **Three ways q moves the force.**
  - q moves C(q), the lever arm of J_c;
  - q moves phi(q) and n(q), hence f;
  - q moves the material-point slip at C, hence dissipation and friction.
- **Isolating the lever arm.** Applying the same f(q, v) at body-fixed points frozen at the nominal C changes
  d(J^T f)/dq by 0.2 % for both joints (4.917 vs 4.917, 5.984 vs 5.984). For sphere-sphere contact the normal force
  passes through both centres, so only friction feels the lever arm: C moves about 57 um per mrad. The geometric
  terms (phi, n) dominate.

**5. Geometric closest-point motion vs. contacting material-point velocity.**

The contact point moves at |dC/dt| = 0.025 m/s, while the finger's material point at C moves at 0.033 m/s and the
ball's at 0.
- The force law uses only the material-point slip v_AcBc (normal part -0.021 m/s), not dC/dt.
- With the ball spinning, the slip becomes 0.115 m/s while dC/dt is unchanged.

The contact point and the contacting material points are different objects, and IDTO keeps them apart correctly.

## 6. Required fixes and configuration changes

| change | why | where |
|---|---|---|
| QEMU x86-64 binfmt registration on the VM (`tonistiigi/binfmt --install amd64`, per boot) | Drake 1.30.0 has no ARM64 Linux build | VM kernel; not persistent |
| `docker/ros2_humble/{Dockerfile,docker-compose.yml}`: ros:humble-ros-base amd64, Drake 1.30.0 in `/opt/drake`, `install_prereqs`, `libgflags-dev`, `scipy==1.12`, `mesa-utils`; source mounted, build in the named volume `idto_build`; X11/GPU wiring as `ros_jazzy` | Humble container as requested; slim (2.4 GB) because the VM had 8.6 GB free | fork, branch `eval/humble-x86` |
| none to IDTO's source for building | builds as-is on 22.04 + Drake 1.30 | |
| MPC timing and closed-loop state logging (`examples/mpc_controller.{h,cc}`, `examples/example_base.cc`) | per-update latency, sim/wall, outcomes | `instrumentation.diff` |
| `save_mpc_result_as_static_html: true` in spinner, dual_jaco, mini_cheetah, allegro_hand YAMLs | Meshcat recordings to inspect | `yaml_recording.diff`; evaluation only, reverted in the fork |
| `contact_gradient_check` target | section 5 | `examples/allegro_hand/` |
| network on first run | Drake's `PackageMap` downloads RobotLocomotion/models at runtime (Allegro, Jaco) | |

## 7. Remaining blockers and assessment for a multi-sphere WB-ID MPC

**What fits.**

- **Formulation.** IDTO is the formulation in question: decision variables are only q_0..N; v and a follow by finite
  differences; tau(q) comes from inverse dynamics with contact forces as an explicit smooth function of (q, v). There
  are no contact modes, complementarity or force variables.
- **Unactuated DoFs.** These are equality constraints (`equality_constraints: true`) or heavily weighted torques.
- **Many spheres.** Drake's `SceneGraph` signed-distance query handles any number of spheres per link, including
  sphere-sphere pairs between robot links and objects.
- **Moving contacts on spheres.** Section 5 shows they are exact and smooth: the closed-form closest points and the
  midpoint contact are recomputed every evaluation, so contacts slide over link and object spheres with consistent
  derivatives.
- **Solver structure.** Gauss-Newton with a block-pentadiagonal Hessian and a trust region; cost per iteration is
  linear in the horizon.
- **Warm-started MPC.** It exists and runs 1-2 iterations per update; Python bindings exist for the core optimizer.

**Gaps for a whole-body ID MPC.**

1. **Speed: real time natively (section 8).** Under emulation every example misses its deadline by 4-8x; natively
   on ARM64 every MPC example meets its rate with 2.6-7.9x headroom.
   - Derivatives are forward differences by default: nq + 1 inverse-dynamics and geometry evaluations per knot, so the
     cost grows with nq and with the number of sphere pairs.
   - A real-time verdict needs native timing, either on Ubuntu 24.04 ARM64 with Drake ≥ 1.53 (IDTO ported to the
     newer API) or on x86 hardware.
2. **Floating-base derivative bias.** The derivative drops the dN+/dq term along rotating floating bases (section 5,
   up to 6.6 %). This affects humanoid and quadruped bases and manipulated objects; Gauss-Newton tolerates it, but a
   floating-base WB-ID formulation should include it, or use a Lie-group velocity parameterization.
3. **No inequality constraints.** Torque, joint, friction-cone (beyond the smooth model) and self-collision limits are
   absent; only cost terms and the unactuated equality exist. A WB-ID MPC needs torque and joint limits at least,
   added as penalties, barriers or an outer constrained solver.
4. **Contact model is soft and acts at a distance.**
   - Stiffness is task-tuned (100-2000 N/m) and the force is nonzero up to the threshold distance (15.7 mm for
     Allegro).
   - Forces are point forces at witness-point midpoints, so there are no patches; per-pair stiffness is global
     rather than per geometry.
   - Planning against the hydroelastic simulation works because MPC closes the loop, but force-level accuracy will not
     transfer.
5. **Closed loop is a synchronous simulation.** There is no asynchronous executor: latency is modelled as exactly
   one replan period. A deployment needs the solver in its own thread with plan timestamping, which the examples do
   not provide.
6. **Old pinned Drake.** IDTO pins Drake 1.30 (2024); ARM64 Linux needs 1.53+. The port took four small changes
   (section 8). Drake's newer simulation defaults change some examples' closed-loop outcomes.

**Assessment.** IDTO is a good starting point for the contact formulation and the optimizer structure: smooth
signed-distance contacts between arbitrary sphere sets, verified moving-contact behaviour, and an inverse-dynamics
trajectory optimizer with a warm-started MPC loop.

It is not yet a WB-ID MPC. It needs:
- native timing to establish feasible rates;
- the floating-base derivative fix;
- inequality constraints (torque and joint limits);
- an asynchronous MPC runtime.

Of those, native timing is the next step, since the rate decides whether faster derivatives (autodiff or analytic
contact derivatives on the sphere pairs) are required before anything else.

## 8. Native ARM64: Drake 1.53 port and real-time timing

Branch `eval/arm64`.

**Container.**
- `idto_jazzy` (`docker/jazzy_arm64/`): native ARM64, on top of the `ros_jazzy` image from `ros2_docker`, so it has
  the virgl GPU.
- Ubuntu 24.04, GCC 13.3.0, CMake 3.28.3, Python 3.12.3, numpy 1.26.4.
- Drake 1.53.0 (`211b5ae66d3ac16c5264b0ceea1902309f698901`), the first release with Linux ARM64 binaries.
- Meshcat on host ports 7005-7009.
- Build: 3 min native, against 12.5 min emulated.

**The port.** All changes are version-neutral; the x86 Humble / Drake 1.30 build was rebuilt from the same tree and
still passes 23/23.

| change | why |
|---|---|
| `CMakeLists.txt`: for GCC >= 12, `-Wno-array-bounds -Wno-stringop-overflow` | GCC 13 false positives inside libstdc++'s `memmove` at `-O3` (bug 107852), fatal under `-Werror` |
| `optimizer/trajectory_optimizer.cc`: `drake::ExtractDoubleOrThrow` in `SaveLinesearchResidual` | `AutoDiffXd` is `drake::ad::AutoDiff` since Drake 1.33, with no `operator<<` |
| `optimizer/test/trajectory_optimizer_test.cc`: `plant.GetFloatingBaseBodies()` instead of `body.is_floating()` | deprecated in 1.53; its replacement `is_floating_base_body()` does not exist in 1.30 |
| `examples/yaml_config.h`, `example_base.cc`: YAML option `sim_discrete_contact_approximation` (default `""` = Drake's default) | Drake's default simulation contact solver changed from TAMSI (1.30) to `lagged` SAP |
| `allegro_hand.yaml`, `punyo.yaml`, `hopper.yaml`: `sim_discrete_contact_approximation: "tamsi"` | restores their 1.30 closed-loop behaviour (below) |

**Tests: 23/23 pass.** The example smoke tests run 10-17x faster than emulated. `trajectory_optimizer_test` took
41 s once only because Drake's `PackageMap` downloaded RobotLocomotion/models on first use; run again, every case
takes 13 ms or less.

**MPC timing, native ARM64** (`arm64/examples/<name>/mpc_timing.csv`, the same instrumentation):

| example | target | solve median / p95 / p99 / max [ms] | rate (median) | update median / max [ms] | misses | sim/wall |
|---|---|---|---|---|---|---|
| pendulum | 300 Hz | 1.27 / 1.80 / 2.12 / 4.95 | 789 Hz | 1.51 / 6.69 | 5 / 1500 (0.3 %) | 1.99 |
| acrobot | 300 Hz | 1.65 / 2.07 / 2.38 / 2.91 | 607 Hz | 1.97 / 3.30 | 0 | 1.58 |
| **spinner** | 200 Hz | 1.82 / 2.23 / 2.44 / 3.32 | 549 Hz | 2.23 / 3.95 | 0 | 1.99 |
| hopper | 100 Hz | 2.78 / 3.33 / 4.31 / 5.66 | 360 Hz | 3.32 / 6.68 | 0 | 2.66 |
| jaco_ball | 100 Hz | 3.44 / 3.77 / 4.67 / 7.81 | 291 Hz | 3.85 / 8.50 | 0 | 2.36 |
| jaco | 20 Hz | 13.49 / 14.46 / 15.10 / 17.25 | 74 Hz | 14.73 / 19.16 | 0 | 2.47 |
| **dual_jaco** | 20 Hz | 19.11 / 20.43 / 23.37 / 26.00 | 52 Hz | 20.26 / 27.30 | 0 | 1.00 (throttled) |
| **mini_cheetah** | 60 Hz | 4.13 / 4.48 / 5.30 / 37.02 | 242 Hz | 5.05 / 38.06 | 1 / 600 (0.2 %) | 2.32 |
| **allegro_hand** | 10 Hz | 24.43 / 25.40 / 30.88 / 34.65 | 41 Hz | 26.47 / 37.28 | 0 | 1.00 (throttled) |
| punyo | 15 Hz | 22.32 / 30.40 / 49.33 / 211.61 | 45 Hz | 24.16 / 215.93 | 2 / 150 (1.3 %) | 1.00 (throttled) |

- **Initial full solves** take 0.04-2.7 s, against 0.6-61 s emulated.
- **Speed-up over emulation:** 10-24x per update.
- **Deadline misses** are isolated outliers: a few ms for pendulum, 37 ms for mini_cheetah, 212 ms for punyo.
- **sim/wall of 1.00** means the run is throttled by `sim_realtime_rate: 1`; the others run faster than real time.

**Planning is unchanged; the simulated plant is not.** The initial `dual_jaco` and `allegro_hand` solves on ARM64 /
Drake 1.53 match x86 / Drake 1.30 to about 1e-9 (same cost history, `q[T]` and torques).

The closed-loop outcomes differ because Drake 1.53 simulates contact differently. Its default discrete solver is
`lagged` (SAP), where 1.30 used TAMSI, and its hydroelastic contact also changed. Repeated runs are bit-identical, so
these are deterministic differences:

| example | 1.30 x86 (TAMSI) | 1.53 SAP (`lagged`, default) | 1.53 TAMSI | shipped on `eval/arm64` |
|---|---|---|---|---|
| allegro_hand | 12.2 deg, ball kept (16 mm) | **ball dropped** (1.8 m, 158 deg) | 18.5 deg, 27 mm | TAMSI |
| punyo | ball at (0.19, 0.37, 0.79) | **ball rolled away** (4.2, 6.9, 0.2) | (0.19, 0.37, 0.80) | TAMSI |
| hopper | x -0.375 m | x -0.265 m | x -0.386 m | TAMSI |
| dual_jaco | 4.2 deg, z 0.105 m | 17.5 deg, z 0.105 m | **not lifted** (46.6 deg, z 0.075) | default (SAP) |

The other examples succeed on Drake 1.53's default:

| example | outcome on Drake 1.53 |
|---|---|
| pendulum | upright, error 0.001 rad |
| acrobot | (3.139, 0.010) |
| spinner | 4.56 rad in 5 s |
| jaco_ball | ball y 0.093 m |
| jaco | box at 7.2 deg, (0.475, -0.100, 0.075) |
| mini_cheetah | 3.43 m in 10 s, max tilt 8.8 deg |

dual_jaco does not reproduce 1.30 under either solver. Its arms, box and ground are hydroelastic, so the remaining
difference lies in Drake's hydroelastic simulation between versions, not in IDTO.

**Gradients.** `contact_gradient_check` on Drake 1.53 reproduces section 5, including the dN+/dq bias: 6.64e-2 along
a ball rotation for all three methods.
