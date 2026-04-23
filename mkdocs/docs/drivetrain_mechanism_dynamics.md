# Drivetrain and Mechanism Dynamics Math

This page captures mathematical assumptions for drivetrain and mechanism dynamics as this subsystem grows.

## Symbols

| Symbol | Meaning | Units |
|---|---|---|
| F_x, F_y | Chassis force components | N |
| tau_z | Chassis yaw torque | N*m |
| r_i | Position of wheel/module i relative to CoM | m |
| f_i | Wheel/module force vector | N |
| J_i | Module Jacobian | varies |
| I | Rotational inertia | kg*m^2 |
| theta | Joint angle | rad |
| omega | Angular speed | rad/s |

## Source and scope

Current implementation status:

The following headers currently exist as placeholders in core/driver:

- core/driver/include/frcsim/drivetrains/tank_model.hpp
- core/driver/include/frcsim/drivetrains/mecanum_model.hpp
- core/driver/include/frcsim/drivetrains/swerve_model.hpp
- core/driver/include/frcsim/mechanisms/arm.hpp
- core/driver/include/frcsim/mechanisms/elevator.hpp
- core/driver/include/frcsim/mechanisms/shooter_wheel.hpp

Because APIs are still evolving, this page documents the baseline equations and assumptions to keep future implementations consistent.

Validation tests:

- vendordep/tests/integration_test.cpp

## Modeling intent

The equations here define baseline reduced-order dynamics suitable for real-time simulation. They are not intended to replace high-fidelity multibody identification models.

## Common drivetrain force mapping

For wheel force sum in body frame:

$$
F_b = \sum_i F_{wheel,i}
$$

and body acceleration:

$$
a_b = \frac{F_b}{m}
$$

Yaw torque from wheel forces:

$$
\tau_z = \sum_i (r_i \times F_{wheel,i})_z
$$

Process for implementation:

1. Compute per-wheel traction force subject to motor and friction limits.
2. Transform each force into a common chassis frame.
3. Sum net force and net yaw moment.
4. Integrate chassis state with selected integrator.

## Tank drive baseline

With left/right tractive forces $$F_L, F_R$$ and track half-width b:

$$
F_x = F_L + F_R
$$

$$
\tau_z = b(F_R - F_L)
$$

Assumptions and limitations:

- symmetric wheel-ground contact
- no transient load transfer model
- slip often approximated with simple friction cap

Engineering note: when tuning turn dynamics, check that track-width parameterization matches your geometric convention.

## Mecanum and swerve baseline

Wheel modules map wheel force vectors to chassis wrench:

$$
\begin{bmatrix}F_x\\F_y\\\tau_z\end{bmatrix} = \sum_i J_i^T f_i
$$

where $$J_i$$ is module Jacobian and $$f_i$$ is wheel/module force vector.

Assumptions and limitations:

- wheel slip is typically lumped into friction limits
- scrub and compliance are often under-modeled
- motor saturation and current limits must be included for realism

For swerve, explicitly document steering angle convention and wheel-forward axis definition to avoid sign ambiguity.

## Arm and elevator baseline

Single-joint arm dynamics:

$$
I\ddot{\theta} = \tau_{motor} - \tau_{gravity} - \tau_{friction}
$$

Elevator dynamics:

$$
m\ddot{x} = F_{motor} - mg - F_{friction}
$$

Assumptions and limitations:

- reduced-order 1-DOF models ignore flex
- gearbox backlash and belt compliance may be omitted initially

When these effects are omitted, indicate expected mismatch regimes (for example, direction reversals, near-zero speed hold).

## Flywheel wheel baseline

Rotational speed dynamics:

$$
J\dot{\omega} = \tau_{motor} - \tau_{loss} - \tau_{ball\ interaction}
$$

Assumptions and limitations:

- ball-wheel contact modeled as lumped disturbance torque
- thermal effects and motor nonlinearities often excluded at first pass

Document whether ball interaction is modeled as impulse-like or distributed over contact time.

## Validation checklist

For each implemented model, include tests for:

- sign conventions and frame transforms
- steady-state behavior under constant input
- transient response after step input
- saturation and friction edge cases

- repeatability under fixed dt and repeated test runs

## Documentation requirement for future model pages

Each model page should include:

- equations with symbol definitions
- units table
- assumptions and known limitations
- one numeric worked example
- one test mapping section

- parameter-identification guidance when coefficients are empirical

## Related Pages

- [Physics Reference](physics_reference.md)
- [Integrators](integrators.md)
- [Collision Impulses](collision_impulses.md)
