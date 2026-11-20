# IDTO Evaluation

Recorded evaluation from 2026-11-19, with documentation simplified on 2026-11-20.
Follow the [README](../README.md#installation) for installation and [Running Examples](running_examples.md) for the current test procedure.
The [handoff](handoff.md) summarizes remaining work.

This report describes existing measurements.
No new simulations or benchmarks were run for the documentation update.


## 1. Setup and Tests

The current container is `idto_jazzy`, using Ubuntu 24.04, GCC 13.3, CMake 3.28, Python 3.12, and Drake 1.53.
The VM has eight virtual CPUs and 7.5 GB RAM.
Source and recorded runs are under `~/src/mpc/contact_implicit_mpc/` on the Mac and `/root/src/mpc/contact_implicit_mpc/` in the container.

All 23 registered tests passed.
They cover Python bindings, optimizer calculations, matrix solvers, and example smoke tests.
The example tests run with `--test`, which disables MPC and performs ten iterations.
They do not establish closed-loop task success.

The fork's changes include timing and state logging, a moving-contact gradient checker, compatibility updates for Drake 1.53, and a configurable simulation contact solver.
The optimizer still builds against Drake 1.30.
Python bindings must use the same Drake installation as the C++ library.


## 2. Example Results

The examples first solve a complete trajectory to initialize MPC.
Subsequent MPC updates use one or two warm-started trust-region iterations.
Kuka, Airhockey, and the upside-down Allegro example perform open-loop optimization and playback.

| Example | Recorded outcome with Drake 1.53 |
| --- | --- |
| Pendulum | Upright, final error about 0.001 rad |
| Acrobot | Final joint angles about [3.139, 0.010] rad |
| Spinner | Rotates 4.56 rad in 5 s |
| Hopper | Reaches x = -0.386 m toward the -0.5 m target |
| Jaco ball | Ball reaches y = 0.093 m |
| Jaco | Box orientation error 7.2 degrees; position [0.475, -0.100, 0.075] m |
| Mini Cheetah | Travels 3.43 m in 10 s; maximum tilt 8.8 degrees |
| Allegro hand | Ball retained; final orientation error 18.5 degrees; maximum drift 27 mm |
| Punyo | Ball reaches approximately [0.188, 0.370, 0.796] m |
| Dual Jaco | Box lifted with the default solver, but orientation error remains 17.5 degrees |

All 13 example variants exited successfully in the earlier evaluation against Drake 1.30.
The older open-loop results included Kuka pushing the box to within 1.1 cm and 2.7 degrees of its target, and Airhockey placing the puck within about 3 cm of its target.
Open-loop plan results do not establish physical execution success.

### Contact Solver Differences

The planning model uses IDTO's smooth contact force law.
The simulated plant uses Drake's discrete contact model.
Changes in Drake's simulation can therefore change closed-loop results without changing the initial optimized plan.

Initial Dual Jaco and Allegro plans agree between Drake 1.30 and 1.53 to about 1e-9.
Their closed-loop simulation results differ.

| Example | Drake 1.30 TAMSI | Drake 1.53 default SAP | Drake 1.53 TAMSI | Current setting |
| --- | --- | --- | --- | --- |
| Allegro | Ball retained; 12.2-degree error | Ball dropped | Ball retained; 18.5-degree error | TAMSI |
| Punyo | Ball lifted to z = 0.79 m | Ball rolled away | Ball lifted to z = 0.80 m | TAMSI |
| Hopper | x = -0.375 m | x = -0.265 m | x = -0.386 m | TAMSI |
| Dual Jaco | 4.2-degree error; z = 0.105 m | 17.5-degree error; z = 0.105 m | Not lifted; z = 0.075 m | Default SAP |

Dual Jaco remains an open simulation discrepancy.
Its arms, box, and ground use hydroelastic contact.


## 3. MPC Timing

The recorded per-update measurements are:

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

"Rate (median)" is the reciprocal of median solve latency, not the measured control publication frequency.
Typical solve latency is below each configured period, but the table includes isolated deadline misses.
Initial full solves take approximately 0.04-2.7 seconds.

`mpc_timing.csv` separates solve latency from the complete update, including warm-start interpolation and solution storage.
`solver_stats.csv` records the initial full optimization and must not be used as per-update MPC timing evidence.

The simulation is synchronous and waits for each solve.
A delay block models exactly one update period of latency; it does not model actual variable wall-clock solve delays.
These results establish computational headroom in the tested setup, not a deployment guarantee.


## 4. Moving-contact gradient verification (Allegro, sphere-sphere)

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


## 5. Assessment and Open Issues

IDTO optimizes only configurations.
Velocities and accelerations follow from finite differences, and inverse dynamics computes generalized forces.
Contact forces are smooth functions of configuration and velocity, not independent decision variables.
There is no explicit contact-mode sequence or complementarity constraint.

The normal force uses a smoothed penetration law and a Hunt-Crossley-like dissipation factor.
Tangential force uses regularized Coulomb friction.
Contact acts at the midpoint of the two closest witness points and can be nonzero at small positive gaps.

Unactuated generalized forces can be enforced as equality constraints or penalized through the torque cost.
The example YAML determines which approach is used; Mini Cheetah and Dual Jaco use penalties, while Allegro enables equalities.

The main remaining limitations are:

1. Rotating floating-base derivatives omit the configuration derivative of the velocity map, as measured above.
2. Explicit torque bounds, joint limits, and self-collision inequalities are not implemented.
3. The synchronous example runtime needs asynchronous solving and timestamped policies for deployment.
4. Contact stiffness and smoothing are task-specific, with no finite contact-patch model or per-geometry stiffness.
5. The two-arm box example still needs investigation under the current simulation.

Keep the contact derivative tests, task outcomes, complete-update latency, and deadline misses in future comparisons.
