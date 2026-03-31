# Quaternion (`frcsim::Quaternion`)

This page documents the `Quaternion` struct in the `frcsim` physics library, which represents a quaternion for 3D rotations and provides operations for orientation, interpolation, and vector rotation.

---

## Overview

`Quaternion` is a 4D type (`w, x, y, z`) used for representing 3D rotations. It supports construction from axis-angle, angular velocity, and provides methods for normalization, interpolation, and conversion to rotation matrices.

---

## Members
- **w, x, y, z**: The quaternion components (double)

---

## Key Methods & Operations

### Constructors
- `Quaternion()` — Identity quaternion
- `Quaternion(double w, double x, double y, double z)` — Initialize with values
- `Quaternion(double w, Vector3 v)` — From scalar and vector part

### Norms & Normalization
- `norm2()` — Squared magnitude
- `norm()` — Magnitude
- `normalized()` — Returns normalized quaternion
- `normalize()` — In-place normalization
- `normalizeIfNeeded(eps)` — Normalize if not already unit

### Construction & Conversion
- `fromAxisAngle(axis, angleRad)` — From axis-angle
- `fromAngularVelocity(omega, dt)` — From angular velocity (small angle)
- `toAxisAngle(axis, angleRad)` — To axis-angle
- `toMatrix(m)` — To 3x3 rotation matrix

### Interpolation
- `slerp(a, b, t)` — Spherical linear interpolation

### Operations
- `conjugate()` — Conjugate
- `inverse()` — Inverse
- `operator*` — Quaternion multiplication
- `operator+`, `operator-` — Addition, negation
- `operator==`, `operator!=` — Comparison

### Vector Rotation
- `rotate(v)` — Rotate a vector
- `forward()`, `up()`, `right()` — Standard basis directions

### Utility
- `isIdentity(eps)` — Check if identity
- `hasNaN()` — Check for NaN components
- `operator<<` — Output stream

---

## Example Usage
```cpp
frcsim::Quaternion q = frcsim::Quaternion::fromAxisAngle(frcsim::Vector3(0,1,0), M_PI/2);
frcsim::Vector3 v = q.rotate(frcsim::Vector3(1,0,0));
```

---

*This documentation was generated for the file: `physics-core/include/frcsim/math/quaternion.hpp`*
