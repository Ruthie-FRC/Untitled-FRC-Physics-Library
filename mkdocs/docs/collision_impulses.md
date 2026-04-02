# Collision Impulses and Boundary Math

This page describes the math conventions RenSim uses for boundary interactions and the impulse model intended for contact response.

## Symbols

| Symbol | Meaning | Units |
|---|---|---|
| phi | Signed distance | m |
| n | Contact normal | unit vector |
| e | Coefficient of restitution | dimensionless |
| mu | Friction coefficient | dimensionless |
| j_n | Normal impulse magnitude | N*s |
| j_t | Tangential impulse magnitude | N*s |
| k_n | Effective inverse mass along contact normal | 1/kg |

## Source and scope

Current implementation status:

- Boundary data structures are implemented.
- Collision detector and contact solver headers in core/driver are currently placeholders.
- Tests currently validate boundary configuration and integration presence.

Primary implementation:

- core/driver/include/frcsim/field/boundary.hpp
- core/driver/include/frcsim/contact/collision_detector.hpp
- core/driver/include/frcsim/contact/contact_solver.hpp

Validation tests:

- vendordep/tests/boundary_test.cpp

## Boundary primitives

Boundary types include wall, plane, box, and cylinder.

Each boundary stores:

- position and orientation
- geometry parameters
- restitution and friction coefficients
- behavior mode (rigid-body style or static-constraint style)

## Signed distance convention

For a point p and plane with normal n and point p0:

$$
\phi(p) = n \cdot (p - p_0)
$$

Interpretation:

- $$\phi > 0$$ no penetration
- $$\phi = 0$$ touching
- $$\phi < 0$$ penetration depth is $$-|\phi|$$

This sign convention should be used consistently in collision detection and correction terms.

## Normal impulse (single contact)

Let relative normal velocity before solve be $$v_n^-$$ and coefficient of restitution be e.

For scalar impulse j along normal n:

$$
j = -\frac{(1+e) v_n^-}{k_n}
$$

where $$k_n$$ is the effective inverse mass along the contact normal.

Apply impulse:

$$
\Delta v = \frac{j}{m} n
$$

with rotational coupling handled through contact Jacobian terms in full rigid-body form.

Implementation process (conceptual):

1. Compute relative velocity at contact point.
2. Project onto normal to obtain closing speed.
3. Compute impulse magnitude from restitution and effective mass.
4. Apply equal-and-opposite impulses to involved bodies.

## Tangential friction impulse

Compute candidate tangential impulse $$j_t$$ and clamp by Coulomb limit:

$$
|j_t| \le \mu j_n
$$

- static friction if within limit
- dynamic friction if clamped to limit

The clamp step is critical to prevent non-physical tangential impulse magnitudes.

## Timestep sensitivity and stabilization

Discrete solvers are sensitive to dt. Practical guidance:

- use fixed dt
- prefer several solver iterations over one very large impulse
- add positional correction (baumgarte or split impulse) to reduce drift

Solver tuning should be treated as a numerical method problem, not only a physical coefficient problem.

## Numeric example (normal-only)

Given:

- mass m = 2 kg
- incoming normal velocity $$v_n^- = -3$$ m/s
- restitution e = 0.5
- no rotational coupling so $$k_n = 1/m = 0.5\ \text{kg}^{-1}$$

Then impulse magnitude:

$$
j = -\frac{(1+0.5)(-3)}{0.5} = 9\ \text{N*s}
$$

Velocity change along normal:

$$
\Delta v = j/m = 9/2 = 4.5\ \text{m/s}
$$

Post-collision normal velocity becomes positive (separating).

## Validation in RenSim

Current available checks:

- vendordep/tests/boundary_test.cpp for boundary configuration

When detector/solver implementation is expanded, add tests for:

- restitution sign and magnitude
- friction clamp behavior
- stack stability across timesteps

Also include regression tests for frame/sign conventions in contact normals and impulse directions.

## Related Pages

- [Physics Reference](physics_reference.md)
- [Units and Conventions](units_and_conventions.md)
- [Integrators](integrators.md)
