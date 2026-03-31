# Vector3 (`frcsim::Vector3`)

This page documents the `Vector3` struct in the `frcsim` physics library, which represents a 3D vector and provides a wide range of vector operations for physics simulations.

---

## Overview

`Vector3` is a 3D vector type with double-precision components (`x`, `y`, `z`). It supports arithmetic, geometric, and physics-related operations, and is used throughout the library for positions, velocities, forces, and more.

---

## Members
- **x, y, z**: The vector components (double)

---

## Key Methods & Operations

### Constructors
- `Vector3()` — Zero vector
- `Vector3(double x, double y, double z)` — Initialize with values

### Arithmetic
- `+`, `-`, `*`, `/` (with scalars and vectors)
- Compound assignment: `+=`, `-=`, `*=`, `/=`

### Norms & Normalization
- `norm2()` — Squared magnitude
- `norm()` — Magnitude
- `normalized()` — Returns normalized vector
- `isZero(eps)` — Checks if vector is near zero

### Dot & Cross Product
- `dot(const Vector3&)` — Dot product
- `cross(const Vector3&)` — Cross product

### Utility
- `clamp(min, max)` — Clamp each component
- `lerp(a, b, t)` — Linear interpolation
- `distance(a, b)` — Distance between vectors
- `projectOnto(axis)` — Projection onto another vector
- `reflect(n)` — Reflect across a normal
- `torque(r)` — Torque at a point

### Physics Helpers
- `magnusForce(velocity, omega, k)` — Magnus effect force
- `dragForce(v, Cd, A, rho)` — Drag force
- `dynamicGravity(velocity, spin, g, magnusCoeff, gravityEffect)` — Gravity with Magnus effect
- `tractionForce(normal, frictionCoeff, normalForce)` — Traction/friction force

### Constants
- `zero()` — Zero vector
- `unitX()`, `unitY()`, `unitZ()` — Unit vectors

### Other
- `hasNaN()` — Check for NaN components
- `operator[]` — Element access
- `operator<<` — Output stream

---

## Example Usage
```cpp
frcsim::Vector3 a(1.0, 2.0, 3.0);
frcsim::Vector3 b = a.normalized();
double d = a.dot(b);
frcsim::Vector3 f = frcsim::Vector3::dragForce(a, 0.5, 0.1);
```

---

*This documentation was generated for the file: `physics-core/include/frcsim/math/vector.hpp`*
