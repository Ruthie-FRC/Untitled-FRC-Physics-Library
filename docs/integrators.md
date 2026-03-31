# Integrators (`frcsim::Integrator`)

This page documents the integrators provided in the `frcsim` physics library, which are used for updating positions, velocities, and orientations in physics simulations.

## Overview

The `Integrator` struct provides static methods for several common numerical integration techniques used in real-time physics engines. These methods are designed for use with the library's `Vector3` and `Quaternion` types.

---

## Methods

### 1. `integrateLinear`
**Type:** Semi-Implicit Euler (a.k.a. Symplectic Euler)

- **Usage:**
  - `integrateLinear(position, velocity, acceleration, dt)`
- **Description:**
  - Updates velocity and position using the semi-implicit Euler method, which is stable and widely used in real-time physics.
  - 
    $$
    \text{velocity} \mathrel{+}= \text{acceleration} \times dt \\
    \text{position} \mathrel{+}= \text{velocity} \times dt
    $$

---

### 2. `integrateLinearExplicit`
**Type:** Explicit Euler

- **Usage:**
  - `integrateLinearExplicit(position, velocity, acceleration, dt)`
- **Description:**
  - Updates position and velocity using the explicit Euler method. Less stable than semi-implicit Euler, but useful for debugging or predictor steps.
  - 
    $$
    \text{position} \mathrel{+}= \text{velocity} \times dt \\
    \text{velocity} \mathrel{+}= \text{acceleration} \times dt
    $$

---

### 3. `integrateAngular`
**Type:** Quaternion Integration

- **Usage:**
  - `integrateAngular(orientation, angularVelocity, dt)`
- **Description:**
  - Integrates orientation using angular velocity and quaternion algebra.
  - Quaternion derivative:
    $$
    \dot{q} = 0.5 \cdot \omega_{quat} \cdot q
    $$
    where $\omega_{quat} = (0, \omega_x, \omega_y, \omega_z)$

---

### 4. `integrateAngularVelocity`
**Type:** Angular Velocity Update

- **Usage:**
  - `integrateAngularVelocity(angularVelocity, angularAcceleration, dt)`
- **Description:**
  - Updates angular velocity using angular acceleration.
  - 
    $$
    \omega \mathrel{+}= \alpha \times dt
    $$

---

### 5. `integrateLinearRK2`
**Type:** Runge-Kutta 2nd Order (Midpoint)

- **Usage:**
  - `integrateLinearRK2(position, velocity, acceleration, dt)`
- **Description:**
  - Uses the midpoint (RK2) method for more accurate integration, useful for projectiles or high-speed mechanisms.

---

## See Also
- [`Vector3`](vector.md)
- [`Quaternion`](quaternion.md)
- [Physics Reference](../../docs/physics_reference.md)

---

*This documentation was generated for the file: `physics-core/include/frcsim/math/integrators.hpp`*
