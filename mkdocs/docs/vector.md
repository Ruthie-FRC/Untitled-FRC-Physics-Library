# Vector3 (frcsim::Vector3)

This page explains Vector3 operations as physics tools: geometry, forces, and units.

## Symbols

| Symbol | Meaning | Units |
|---|---|---|
| v | Generic vector quantity | varies |
| F | Force vector | N |
| r | Lever-arm vector | m |
| tau | Torque vector | N*m |
| omega | Angular velocity vector | rad/s |
| mu | Friction coefficient | dimensionless |

## Source and scope

Primary implementation:

- core/driver/include/frcsim/math/vector.hpp

Validation tests:

- vendordep/tests/math_test.cpp
- vendordep/tests/forces_test.cpp
- vendordep/tests/magnus_test.cpp

## Core geometric operations

### Dot product

$$
a \cdot b = |a||b|\cos\theta
$$

Use cases:

- projection magnitude
- measuring alignment
- speed along an axis

### Cross product

$$
a \times b
$$

Use cases:

- torque direction and magnitude
- normal vectors
- Magnus side-force direction

Right-hand rule defines sign.

### Norm and normalization

$$
|v| = \sqrt{v_x^2 + v_y^2 + v_z^2}
$$

Normalized direction:

$$
\hat{v} = \frac{v}{|v|}
$$

Always guard near-zero magnitude.

## Coordinate and frame discipline

Vector operations are only physically meaningful when vectors are expressed in the same frame.

- Correct: $$tau_w = r_w \times F_w$$
- Incorrect: mixing $$r_b$$ with $$F_w$$

If data sources differ by frame, transform first and then compute.

## Projection, reflection, and interpolation

### Projection onto axis

$$
\operatorname{proj}_{a}(v) = a \frac{v \cdot a}{a \cdot a}
$$

### Reflection about normal n

$$
v_{refl} = v - 2(v \cdot n)n
$$

Expected when n is unit length.

### Linear interpolation

$$
\operatorname{lerp}(a,b,t) = a(1-t) + bt
$$

## Torque helper

Vector3 torque helper implements:

$$
tau = r \times F
$$

Units:

- r in m
- F in N
- tau in N*m

Worked example:

- $$r = (1,0,0)$$ m
- $$F = (0,10,0)$$ N
- $$\tau = (0,0,10)$$ N*m

This sign and magnitude match integration tests for force-at-point behavior.

Interpretation:

- magnitude: $$|\tau| = |r||F|\sin\theta$$
- direction: right-hand rule perpendicular to the r/F plane

## Physics helper formulas and units

### magnusForce(velocity, omega, k)

Equation:

$$
F_m = k(\omega \times v)
$$

Units:

- velocity: m/s
- omega: rad/s
- k: tuned coefficient
- result: N-equivalent force term

### dragForce(v, Cd, A, rho)

Equation:

$$
F_d = -\frac{1}{2}\rho C_d A |v|^2 \hat{v}
$$

Units:

- v: m/s
- Cd: dimensionless
- A: m^2
- rho: kg/m^3
- force: N

Detailed variant exposes diagnostics such as dynamic pressure, linear drag term, and total drag magnitude.

### dynamicGravity(velocity, spin, g, magnusCoeff, gravityEffect)

Equation form:

$$
a = (0,0,-g*gravityEffect) + magnusForce(velocity, spin, magnusCoeff)
$$

Units:

- returned vector is acceleration-like term (m/s^2 interpretation in integration context)

### tractionForce(normal, frictionCoeff, normalForce)

Equation:

$$
F_t = normal * (\mu N)
$$

Units:

- normal: unitless direction
- frictionCoeff (mu): dimensionless
- normalForce (N): newtons
- result: N

Note: this helper applies force along supplied normal direction. In full contact models, friction is typically tangential to the normal.

## Free-body style examples

Example 1: wheel-ground traction on flat ground

- normal direction: +Z
- mu = 0.8
- normal force = 100 N
- traction magnitude = 80 N in +Z direction from helper form

Example 2: drag and thrust along X

- thrust = +5 N on X
- drag = -1.2 N on X
- net force = +3.8 N on X
- acceleration = net/mass

## Common mistakes

- using unnormalized normals in reflection/traction
- reversing cross-product operand order
- mixing force (N) and acceleration (m/s^2)
- applying drag in same direction as velocity

## Validation in RenSim

- vendordep/tests/math_test.cpp: dot/cross/norm/normalization checks
- vendordep/tests/forces_test.cpp: drag diagnostics and force accumulation
- vendordep/tests/magnus_test.cpp: cross-product sign for Magnus direction

## Related Pages

- [Units and Conventions](units_and_conventions.md)
- [Aerodynamics Math](aerodynamics.md)
- [Magnus Effect Math](magnus.md)
- [Matrix3](matrix.md)
