# Inverse Dynamics Trajectory Optimization

Implements the contact-implicit trajectory optimization algorithm described in [Inverse Dynamics Trajectory Optimization for Contact-Implicit Model Predictive Control](https://idto.github.io/) by Vince Kurtz, Alejandro Castro, Aykut Özgün Önol, and Hai Lin.
See the [paper](https://arxiv.org/abs/2309.01813) for the formulation.

<img src="img/mini_cheetah.png" width="200"><img src="img/dual_jaco.png" width="200"><img src="img/allegro.png" width="200">

<img src="img/spinner.png" width="200"><img src="img/jaco_ball.png" width="200"><img src="img/hopper.png" width="200">

<img src="img/jaco.png" width="200"><img src="img/acrobot.png" width="200"><img src="img/punyo.png" width="200">


## Build and Test

Use the existing `idto_jazzy` container with Ubuntu 24.04 and Drake 1.53.
The examples use Drake simulation and Meshcat visualization, not ROS launch or MuJoCo.


### 1. Enter the Container

From the Mac:

```bash
ssh parallels@10.211.55.17
docker start idto_jazzy
docker exec -it idto_jazzy bash --noprofile --norc
```

Do not source ROS before building.
The Mac source directory is available at `/root/src/mpc/contact_implicit_mpc/idto` inside the container.


### 2. Build

Inside the container:

```bash
cmake --build /root/idto_build -j 8
```

If the build directory has not been configured, configure it first:

```bash
cmake -S /root/src/mpc/contact_implicit_mpc/idto -B /root/idto_build
cmake --build /root/idto_build -j 8
```

Rebuild after changing an example YAML.
The executables read configuration copies from the build directory.


### 3. Run an Example

Use a separate working directory for each run to preserve its CSV files:

```bash
RUN=/root/src/mpc/contact_implicit_mpc/idto_runs/manual_$(date +%Y%m%d_%H%M%S)/mini_cheetah
mkdir -p "$RUN"
cd "$RUN"
/root/idto_build/examples/mini_cheetah/mini_cheetah
```

Open [Meshcat](http://10.211.55.17:7005) in the Mac browser while the example runs.
The example first computes an initial trajectory, then starts MPC.
Run one example at a time.
Press `Ctrl+C` to stop it.


### Available Examples

Choose one command and use its own working directory:

```bash
# Finger flicking a spinner
/root/idto_build/examples/spinner/spinner

# Hand rotating a ball
/root/idto_build/examples/allegro_hand/allegro_hand

# Arm rolling a ball
/root/idto_build/examples/jaco_ball/jaco_ball

# Arm manipulating a box
/root/idto_build/examples/jaco/jaco

# Two arms manipulating a box
/root/idto_build/examples/dual_jaco/dual_jaco

# Quadruped locomotion
/root/idto_build/examples/mini_cheetah/mini_cheetah

# Hopping robot
/root/idto_build/examples/hopper/hopper

# Compliant humanoid lifting a ball
/root/idto_build/examples/punyo/punyo

# Pendulum and acrobot swing-up
/root/idto_build/examples/pendulum/pendulum
/root/idto_build/examples/acrobot/acrobot

# Open-loop trajectory optimization and playback
/root/idto_build/examples/kuka/kuka
/root/idto_build/examples/airhockey/airhockey
/root/idto_build/examples/allegro_hand/allegro_hand --upside_down
```

Most examples run closed-loop MPC.
The last three commands perform open-loop optimization and playback rather than closed-loop simulation.
The current two-arm box example does not reproduce the older Drake version's result; see the [evaluation report](docs/evaluation_report.md).


### 4. Run Registered Tests

Inside the container:

```bash
ctest --test-dir /root/idto_build --output-on-failure
```

The registered example tests are smoke tests.
Their `--test` flag disables MPC and runs only ten optimizer iterations.


### 5. Run a Batch

The existing workspace helper saves separate logs and CSV files per example:

```bash
OUT=/root/src/mpc/contact_implicit_mpc/idto_runs/manual_$(date +%Y%m%d_%H%M%S) \
bash /root/src/mpc/contact_implicit_mpc/idto_runs/run_examples.sh \
600 spinner mini_cheetah allegro_hand
```

The timeout is 600 seconds per example.
Omit the example names to run all examples.
The helper is in the adjacent `idto_runs` directory, not part of this Git repository.


## Configuration and Outputs

Edit `examples/<name>/<name>.yaml`, then rebuild before running.

| Setting | Meaning |
| --- | --- |
| `mpc` | Closed-loop MPC when true; a single trajectory optimization otherwise |
| `time_step`, `num_steps` | Optimization timestep and horizon length in steps |
| `mpc_iters` | Optimizer iterations per MPC update |
| `controller_frequency` | Requested MPC update frequency |
| `num_threads` | Parallel computation threads |
| `sim_time`, `sim_realtime_rate` | Simulation duration and playback speed limit |
| `save_mpc_result_as_static_html` | Save a browser-viewable MPC recording |
| `static_html_filename` | Saved recording path |

The live Meshcat server closes when the example exits.
To retain playback, enable `save_mpc_result_as_static_html` and set a unique `static_html_filename` before rebuilding.

| Output | Contents |
| --- | --- |
| `mpc_timing.csv` | Per-update solve and full controller-update latency |
| `sim_state.csv` | Simulated state sampled at 100 Hz |
| `solver_stats.csv` | Initial trajectory optimization statistics, not MPC update timing |
| `run.log` | Console output when using the batch helper |

The first use of some models requires network access for Drake's model downloads.


## Further Reading

- [Evaluation report](docs/evaluation_report.md): results, contact gradients, timing, and limitations.
- [Handoff](docs/handoff.md): current implementation status and remaining work.
- [Evaluation plan](docs/evaluation_plan.md): checks for future changes.

The core optimizer also has Python bindings under `python_bindings` and examples under `python_examples`.
Use the same Drake installation for C++ and `pydrake`.
The bindings do not perform all the dimension checks of the C++ interface.
