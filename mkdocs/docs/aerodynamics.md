# Aerodynamics Math

This page explains drag equations used by RenSim and how projected area and coefficients affect force.

## Symbols

| Symbol | Meaning | Units |
|---|---|---|
| rho | Air density | kg/m^3 |
| C_d | Drag coefficient | dimensionless |
| A | Reference/projected area | m^2 |
| v | Speed magnitude | m/s |
| v_hat | Velocity unit direction | dimensionless |
| q | Dynamic pressure | Pa |
| k_1 | Linear drag coefficient (Seperate from C_d) | N/(m/s) |
| k_2 | Quadratic drag coefficient | N/(m/s)^2 |

## Source and scope

Primary implementation:

- core/driver/include/frcsim/aerodynamics/drag_model.hpp
- core/driver/include/frcsim/math/vector.hpp
- core/driver/include/frcsim/rigidbody/rigid_body.hpp

Validation tests:

- vendordep/tests/forces_test.cpp

## Quadratic drag

RenSim uses the standard magnitude model:

$$
|F_d| = \frac{1}{2} \rho C_d A v^2
$$

where:

- rho: air density (kg/m^3)
- C_d: drag coefficient (dimensionless)
- A: reference area (m^2)
- v: speed (m/s)

Direction is opposite velocity:

$$
F_d = -|F_d| \hat{v}
$$

Physical interpretation:

- drag magnitude grows with speed squared
- force direction always opposes motion through fluid

## Combined linear + quadratic drag

Detailed diagnostics in Vector3 include both terms:

$$
|F_d| = k_1 v + k_2 v^2
$$

with:

- $$k_1$$ in N/(m/s)
- $$k_2 = \frac{1}{2}\rho C_d A$$ in N/(m/s)^2

This helps tune low-speed damping and high-speed aerodynamic loss independently.

Implementation interpretation:

- linear term dominates as v approaches 0
- quadratic term dominates at higher speed

## Dynamic pressure

RenSim reports dynamic pressure:

$$
q = \frac{1}{2} \rho v^2
$$

and uses:

$$
|F_{d,quad}| = q C_d A
$$

## Projected area from body geometry

For rigid bodies, area can be inferred from configured geometry and velocity direction.

- Sphere: $$A = \pi r^2$$
- Box: weighted projected area from dimensions and direction cosines
- Cylinder: blend of end-cap and side projection based on axis alignment

Cylinder projected-area behavior is validated in forces tests using both local-axis and world-axis setters.

For a cylinder aligned by axis vector a and velocity direction d:

- end-cap contribution scales with alignment magnitude |a dot d|
- side contribution scales with transverse component sqrt(1 - (a dot d)^2)

## Numeric example

Given:

- rho = 1.225 kg/m^3
- C_d = 0.47
- A = 0.01 m^2
- v = 10 m/s

Then:

$$
q = 0.5 * 1.225 * 10^2 = 61.25\ \text{Pa}
$$

$$
|F_d| = q C_d A = 61.25 * 0.47 * 0.01 = 0.287875\ \text{N}
$$

Direction is opposite the velocity vector.

## Drag vs effective gravity

DragModel exposes comparison against effective gravity acceleration:

$$
\text{ratio} = \frac{|a_{drag}|}{|g_{eff}|}
$$

Interpretation:

- ratio << 1: drag is minor
- ratio ~ 1: drag is comparable to gravity
- ratio > 1: drag dominates trajectory shape

This ratio is useful when deciding whether gravity-only approximations are acceptable for a gamepiece trajectory.

## Tuning guidance

- Start with physically plausible C_d and A.
- Add linear term only if low-speed damping is under-modeled.
- Keep signs frame-consistent before tuning coefficients.
- Validate with speed-decay and drop tests, not just one timestep.

## Validation in RenSim

Use:

- vendordep/tests/forces_test.cpp

Check:

- drag direction is opposite velocity
- detailed diagnostics fields are populated and positive
- invalid input fails closed to zero force

Recommended additional check:

- compare simulated terminal-speed trend against expected drag balance for representative masses

## Related Pages

- [Vector3](vector.md)
- [Physics Reference](physics_reference.md)
- [Magnus Effect Math](magnus.md)
- [Units and Conventions](units_and_conventions.md)
