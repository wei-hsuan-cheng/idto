# Handoff: IDTO Evaluation

Updated 2026-11-20.
For a fresh installation, follow the [README](../README.md#installation).
For the existing environment and example commands, follow [Running Examples](running_examples.md).
The [evaluation report](evaluation_report.md) contains the recorded results.


## Workspace

| Item | Location |
| --- | --- |
| Source | Mac: `~/src/mpc/contact_implicit_mpc/idto`; container: `/root/src/mpc/contact_implicit_mpc/idto` |
| Runs | Adjacent `idto_runs/`, containing logs, CSVs, recordings, and `run_examples.sh` |
| VM | `ssh parallels@10.211.55.17`; the Mac's `~/src` is mounted at `/media/psf/Home/src` |
| Container | `idto_jazzy`: Ubuntu 24.04, ROS 2 Jazzy, Drake 1.53 in `/opt/drake` |
| Build | `/root/idto_build`, stored in a persistent Docker volume |
| Visualization | Meshcat at `http://10.211.55.17:7005` while an example runs |

The parent `contact_implicit_mpc` directory is not a Git repository.
The source repository is the fork `wei-hsuan-cheng/idto`, with `upstream` pointing to Toyota Research Institute.


## Current Implementation

IDTO optimizes only the configuration trajectory.
Velocities and accelerations follow from finite differences.
Drake supplies geometry, kinematics, and rigid-body inverse dynamics.
IDTO supplies a smooth signed-distance contact force law and a Gauss-Newton optimizer.
The MPC loop shifts the previous trajectory and runs one or two trust-region iterations per update.

The planning and simulation contact models are different.
Planning uses IDTO's smooth compliant contact.
Simulation uses Drake contact, with the YAML selecting the discrete contact approximation.

Timing and state logging are implemented in `examples/mpc_controller.{h,cc}` and `examples/example_base.cc`.
The moving-contact derivative checker is `examples/allegro_hand/contact_gradient_check.cc`.


## Recorded Results

- All 23 registered tests passed.
- Existing examples were evaluated with logs and CSVs preserved in the adjacent runs directory.
- Typical solve latency is below the requested update period, but isolated deadline misses exist.
- Sphere-sphere witness points match the closed-form geometry.
- Hand-joint derivatives match independent central differences to about 2e-7 with the configured forward-difference method.
- Rotating floating-base derivatives have a systematic error of up to about 6.6 percent because the velocity derivatives omit the configuration derivative of the velocity map.
- Allegro, Punyo, and Hopper use `sim_discrete_contact_approximation: "tamsi"` to retain their earlier simulation behavior.
- The two-arm box example does not reproduce the earlier result under either tested contact solver.

These are recorded evaluation findings, not new tests performed during the documentation cleanup.


## Practical Notes

- Do not source ROS before building.
- Rebuild after editing source YAMLs because the executable reads build-tree copies.
- Reconfigure CMake after adding a target.
- Use a unique working directory per run to avoid overwriting CSVs.
- The first run of some examples downloads models.
- Geometry names are scoped by model instance; match contact pairs by `GeometryId`.
- Meshcat closes when the executable exits; enable an HTML recording before rebuilding if playback is needed.
- `solver_stats.csv` records the initial solve; use `mpc_timing.csv` for MPC timing.
- `--test` is only a smoke test and does not run MPC.


## Remaining Work

1. Correct the rotating floating-base derivatives, including the missing configuration derivative of the velocity map, and rerun the gradient checker.
2. Investigate the two-arm box simulation discrepancy without confusing simulation contact behavior with optimizer changes.
3. Add explicit torque and joint-limit handling before treating this as a whole-body controller.
4. Add asynchronous solving, observation and policy timestamps, and stale-policy handling for deployment.
5. Rebenchmark after each functional change, recording tail latency and deadline misses rather than only reciprocal median solve time.
