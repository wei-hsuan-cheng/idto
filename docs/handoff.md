# Handoff: IDTO evaluation

State as of 2026-11-19. The plan is [evaluation_plan.md](evaluation_plan.md); results are in
[evaluation_report.md](evaluation_report.md).

## Where things are

| what | where |
|---|---|
| fork | `github.com/wei-hsuan-cheng/idto`: `main` = upstream `de0629c` + the plan (`39c5022`); `eval/arm64` = the x86 evaluation, then the Drake 1.53 port and native results (the tree builds on both Drakes; the separate `eval/humble-x86` branch was deleted 2026-11-20) |
| source | Mac `~/src/mpc/contact_implicit_mpc/idto` (the parent folder is not a git repo); remote `upstream` = TRI |
| runs | Mac `~/src/mpc/contact_implicit_mpc/idto_runs/`: `run_examples.sh`, `examples/<name>/` (`run.log`, `mpc_timing.csv`, `sim_state.csv`, `solver_stats.csv`, some `meshcat_recording.html`), `gradient/`, `tests/`, `instrumentation.diff`, `yaml_recording.diff` |
| VM | Parallels Ubuntu 24.04 aarch64, `ssh parallels@10.211.55.17`; the Mac's `~/src` is `/media/psf/Home/src`; about 5.0 GB disk free |
| container (x86) | **removed 2026-11-20** (container, image, volume and the QEMU registration). It was `idto_humble`: ROS 2 Humble, Ubuntu 22.04, x86-64 under QEMU, Drake 1.30.0; recreate it from `docker/ros2_humble/` (below) |
| container (native) | `idto_jazzy`: ROS 2 Jazzy, Ubuntu 24.04, **native ARM64**, Drake 1.53.0 in `/opt/drake`, built on the `ros_jazzy` image (virgl GPU); build in the volume `idto_build_arm64`; Meshcat at `http://10.211.55.17:7005`; branch `eval/arm64` |

## Resume

Native ARM64 (real-time timing):

```bash
docker start idto_jazzy && docker exec -it idto_jazzy bash      # no QEMU needed
cmake --build /root/idto_build -j 8                              # from `bash -c`, without sourcing ROS
OUT=/root/src/mpc/contact_implicit_mpc/idto_runs/arm64/examples bash /root/src/mpc/contact_implicit_mpc/idto_runs/run_examples.sh 600 [names...]
```

x86 Humble (IDTO's supported setup; removed, recreate it):

```bash
# on the VM: x86 emulation (lost on every reboot), then the image (about 15 min) and the container
docker run --privileged --rm tonistiigi/binfmt --install amd64
cd /media/psf/Home/src/mpc/contact_implicit_mpc/idto
docker compose -f docker/ros2_humble/docker-compose.yml build && docker compose -f docker/ros2_humble/docker-compose.yml up -d
docker exec idto_humble bash -c "cmake -S /root/src/mpc/contact_implicit_mpc/idto -B /root/idto_build"   # then build as below
docker exec -it idto_humble bash
# inside
cmake --build /root/idto_build -j 4                     # after any source or YAML edit
ctest --test-dir /root/idto_build --output-on-failure   # 23 tests
cd /root/src/mpc/contact_implicit_mpc/idto_runs/mine && /root/idto_build/examples/mini_cheetah/mini_cheetah
bash /root/src/mpc/contact_implicit_mpc/idto_runs/run_examples.sh 3600 [names...]
/root/idto_build/examples/allegro_hand/contact_gradient_check --ball_spin 2 --gradients_method autodiff
```

## Done

- Humble x86 container (`docker/ros2_humble/`). IDTO builds with no source changes, in 12.5 min emulated.
- Tests: 23/23 ctest pass. The example tests are smoke tests only: `--test` means no MPC and 10 iterations.
- Examples: all 13 runs exit 0 and reach their tasks (report section 3).
- MPC timing instrumentation (`mpc_timing.csv`, sim/wall, `sim_state.csv`). Emulated rates are 1/4-1/8 of the
  configured ones, with 100 % deadline misses (report section 4).
- Gradient check (`contact_gradient_check`):
  - sphere-sphere witness points are exact;
  - the configured derivatives match central differences to about 2e-7;
  - **all gradient methods drop dN+/dq**, giving 0.1-6.6 % error along a rotating floating base.

## Traps

- **QEMU registration is lost on VM reboot.** Without it the x86 container fails with "exec format error".
- **A source YAML edit needs a rebuild.** Binaries read the build-tree copy.
- **A new CMake target needs an explicit reconfigure** (`cmake -S ... -B ...`). `make <new-target>` fails first.
- **Drake scopes geometry names** (`ball::ball_collision`). Match pairs by `GeometryId`.
- **First runs need network.** Drake's `PackageMap` downloads RobotLocomotion/models (Allegro, Jaco).
- **No GPU in the x86 container.** QEMU does not pass the virtio-GPU through, so rendering is `llvmpipe`. IDTO does
  not need a GPU; Meshcat renders in the browser.
- **Meshcat only lives while an example runs.** For a replay, set `save_mpc_result_as_static_html: true` in the YAML.

## Open

1. **Native ARM64 timing: done** (report section 8). Every MPC example is real time with 2.6-7.9x headroom on Drake
   1.53. dual_jaco's closed loop does not reproduce Drake 1.30 under either solver (hydroelastic simulation changes).
2. **Floating-base derivative.** Add the dN+/dq term, or switch to a Lie-group velocity parameterization.
3. **Constraints and runtime for a WB-ID MPC.** Inequality constraints (torque and joint limits) and an asynchronous
   MPC runtime.
