# Integrators (frcsim::Integrator)

This page explains the numerical integration methods used in JSim, their error properties, and practical method-selection tradeoffs.

## Symbols

| Symbol | Meaning | Units |
|---|---|---|
| x | Position | m |
| v | Linear velocity | m/s |
| a | Linear acceleration | m/s^2 |
| q | Orientation quaternion | dimensionless |
| omega | Angular velocity | rad/s |
| alpha | Angular acceleration | rad/s^2 |
| dt | Fixed timestep | s |

## Source and scope

Primary implementation:

- core/driver/include/frcsim/math/integrators.hpp

Validation tests:

- vendordep/tests/integration_test.cpp

## State equations

Linear dynamics are integrated from:

$$
\dot{x} = v, \quad \dot{v} = a
$$

Angular dynamics use quaternion kinematics:

$$
\dot{q} = \frac{1}{2} \omega_q q
$$

with $$\omega_q = (0, \omega_x, \omega_y, \omega_z)$$.

## Method comparison

| Method | Order (global) | Cost | Typical stability in real-time sims | Notes |
|---|---|---|---|---|
| Explicit Euler | 1 | Low | Weak for stiff dynamics | Good for debugging and simple predictors |
| Semi-Implicit Euler | 1 | Low | Better for energy behavior in mechanics | Default-style choice in many engines |
| RK2 (midpoint) | 2 | Medium | Better accuracy per step | Useful for fast projectiles and smoother trajectories |

## Error model summary

- Explicit Euler local truncation error: O(dt^2), global error: O(dt).
- Semi-implicit Euler local truncation error: O(dt^2), global error: O(dt).
- RK2 midpoint local truncation error: O(dt^3), global error: O(dt^2).

Practical implication: reducing dt improves all methods, but RK2 usually achieves smaller trajectory error at equal dt.

## Linear integration methods

### Semi-Implicit Euler (integrateLinear)

Update order:

$$
v_{n+1} = v_n + a_n dt
$$

$$
x_{n+1} = x_n + v_{n+1} dt
$$

Why it is commonly used:

- more stable than explicit Euler for many mechanical systems
- better long-run behavior for velocity-coupled motion

### Explicit Euler (integrateLinearExplicit)

Update order:

$$
x_{n+1} = x_n + v_n dt
$$

$$
v_{n+1} = v_n + a_n dt
$$

Use when:

- you need the old-state predictor style update
- you are debugging step ordering

### RK2 midpoint (integrateLinearRK2)

JSim midpoint-style update:

$$
v_{mid} = v_n + a_n \frac{dt}{2}
$$

$$
x_{n+1} = x_n + v_{mid} dt
$$

$$
v_{n+1} = v_n + a_n dt
$$

Use when:

- projectile motion needs less trajectory error at same dt
- you want better accuracy without moving to full RK4 cost

## Angular integration

### Angular velocity update (integrateAngularVelocity)

$$
\omega_{n+1} = \omega_n + \alpha_n dt
$$

### Quaternion update (integrateAngular)

JSim computes:

$$
dq = 0.5 (\omega_q q)
$$

$$
q_{n+1} = q_n + dq dt
$$

and then normalizes if needed.

### Why q integration is written this way

Quaternion derivative is linear in angular velocity and quaternion state. A forward update is computationally cheap and robust in real-time loops when followed by normalization.

Why normalization matters:

- floating-point drift causes $$|q| \neq 1$$ over time
- non-unit quaternions produce scaling/rotation artifacts

## Error and timestep guidance

- Explicit and semi-implicit Euler are first-order globally: error scales approximately with dt.
- RK2 is second-order globally: error scales approximately with $$dt^2$$.
- Halving dt usually improves Euler methods notably, but cost doubles.

Practical guidance for FRC-like simulation loops:

- keep fixed dt
- start near 0.01 s to 0.02 s for many robot dynamics loops
- reduce dt for stiff spring contacts, high-speed impacts, or high angular rates

Rule-of-thumb process:

1. Start with fixed dt used by your runtime loop.
2. Verify stability in worst-case contact and high-rate spin scenes.
3. If unstable, reduce dt before changing physical coefficients.
4. If stable but inaccurate, prefer RK2 for the affected subsystem.

## Energy behavior and drift expectations

- Explicit Euler can over-inject or dissipate energy depending on system.
- Semi-implicit Euler often gives better qualitative energy behavior in rigid-body motion.
- RK2 typically reduces trajectory drift but still needs appropriate dt.

## Choose this method when

- choose Semi-Implicit Euler for general real-time rigid-body stepping
- choose Explicit Euler for diagnostics and predictor workflows
- choose RK2 when projectile or fast mechanism accuracy needs improvement at same dt

## Worked free-fall comparison (one step)

Given $$x_0=0$$, $$v_0=0$$, $$a=-9.81$$, $$dt=0.01$$:

- Explicit Euler:
	- $$x_1 = x_0 + v_0 dt = 0$$
	- $$v_1 = v_0 + a dt = -0.0981$$
- Semi-implicit Euler:
	- $$v_1 = -0.0981$$
	- $$x_1 = x_0 + v_1 dt = -0.000981$$

This illustrates why step ordering affects short-horizon position estimates.

## Validation in JSim

Use vendordep/tests/integration_test.cpp to verify:

- explicit vs semi-implicit ordering differences
- expected free-fall position/velocity bounds over fixed simulated time
- damping trends and kinematic body behavior

## Related Pages

- [Units and Conventions](units_and_conventions.md)
- [Vector3](vector.md)
- [Quaternion](quaternion.md)
- [Matrix3](matrix.md)
