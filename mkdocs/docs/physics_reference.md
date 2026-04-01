# Physics Reference

This page summarizes the physical models currently represented in RenSim.

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

RenSim supports force models including:

- Gravity
- Springs
- Motors/actuators
- Aerodynamic terms (drag, Magnus effect)

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
