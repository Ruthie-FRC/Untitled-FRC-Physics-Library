# Units and Conventions

This page defines shared notation, units, frame conventions, and sign rules used across all JSim math documentation.

## Notation standard

- Scalars: italic lower-case (for example, m, t, dt).
- Vectors: bold lower-case in equations (for example, v, F, omega).
- Matrices/tensors: upper-case (for example, R, I).
- Body-frame quantities: subscript b.
- World-frame quantities: subscript w.

## Base units

- Position: meters (m)
- Velocity: meters per second (m/s)
- Acceleration: meters per second squared (m/s^2)
- Force: newtons (N)
- Torque: newton-meters (N*m)
- Mass: kilograms (kg)
- Time step: seconds (s)
- Angle: radians (rad)
- Angular velocity: radians per second (rad/s)
- Angular acceleration: radians per second squared (rad/s^2)

## Core vector conventions

- Dot product: $$a \cdot b = |a||b|\cos\theta$$
- Cross product: right-hand rule
- Torque from lever arm and force: $$\tau = r \times F$$

## Matrix and rotation conventions

- Vectors are treated as column vectors.
- Transform convention: $$v_w = R_{wb} v_b$$.
- Rotation inverse for orthonormal R: $$R^{-1} = R^T$$.
- Proper rotation checks:
  - $$R^T R = I$$
  - $$\det(R)=1$$

## Frame naming

- Body frame quantities use subscript b.
- World frame quantities use subscript w.
- Example transform:
  - $$v_w = R_{wb} v_b$$

## Quaternion convention

- Quaternion is ordered as (w, x, y, z).
- Vector rotation uses:
  - $$v' = q v q^{-1}$$
- q and -q encode the same orientation.

## Time-stepping conventions

- dt is always in seconds.
- Fixed dt is preferred for deterministic behavior.
- Semi-implicit Euler order: velocity first, then position.
- Explicit Euler order: position first, then velocity.

## Integrator conventions

- Fixed dt is preferred for repeatability.
- Semi-implicit Euler updates velocity before position.
- Explicit Euler updates position before velocity.

## Aerodynamics conventions

- Drag force points opposite velocity direction.
- Dynamic pressure:
  - $$q = \frac{1}{2}\rho v^2$$
- Drag magnitude (quadratic term):
  - $$|F_d| = q C_d A$$

## Boundary and collision sign conventions

- Contact normal points from surface toward permitted space.
- Positive separation means no penetration.
- Impulses along normal oppose closing velocity.

## Documentation quality checklist

For each equation in this doc set:

- Define each symbol once.
- Include units for each term.
- State the frame of each vector quantity.
- Specify sign convention when direction matters.

## Common mistakes checklist

- Mixing body and world frame terms in one equation.
- Using degrees in trig where radians are expected.
- Using force where acceleration is required (missing divide by mass).
- Forgetting sign direction for drag and friction.
