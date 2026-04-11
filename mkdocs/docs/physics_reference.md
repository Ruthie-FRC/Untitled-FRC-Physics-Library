# Physics Reference

This page summarizes the physical models currently represented in JSim.

## Rigid-Body Dynamics

Core rigid-body updates are based on Newton-Euler mechanics:

$$
\mathbf{F} = m\mathbf{a}, \qquad \boldsymbol{\tau} = \mathbf{I}\boldsymbol{\alpha}
$$

Linear and angular state are integrated per timestep using methods documented in [Integrators](integrators.md).

## Rotation Representation

Orientations are represented with quaternions to avoid gimbal lock and improve numerical stability.

Quaternion state updates follow:

$$
\dot{q} = \frac{1}{2}\,\omega_q\,q
$$

See [Quaternion](quaternion.md) for API details.

## Forces

JSim supports force models including:

- Gravity
- Springs
- Motors/actuators
- Aerodynamic terms (drag, Magnus effect)

Aerodynamic drag is modeled with a projected cross-sectional area. For rigid bodies that define aerodynamic geometry, the engine estimates frontal area from the body's shape and orientation before applying the drag force.

Drag can also be compared against effective gravity on a per-body basis, which is useful for judging whether aerodynamic forces are small, comparable, or dominant relative to weight.

The current body geometry support covers custom reference areas and simple box/sphere/cylinder approximations.
For cylinders, the local axis can be set directly through a convenience enum API to avoid manual axis-vector setup.
Cylinder axis can also be supplied in world coordinates; the engine converts it into local geometry space using the current body orientation.

## Numerical Considerations

- Use fixed timestep simulation where possible.
- Select integration methods based on stability and performance requirements.
- Validate material and damping parameters against expected physical behavior.

## Current Scope

The engine is under active development. Some advanced subsystems are still being expanded.

## Related Pages

- [API Usage](api_usage.md)
- [Architecture](architecture.md)
- [Integrators](integrators.md)
