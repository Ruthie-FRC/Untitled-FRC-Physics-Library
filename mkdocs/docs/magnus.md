# Magnus Effect Math

This page explains the spin-induced side force model used by JSim.

## Symbols

| Symbol | Meaning | Units |
|---|---|---|
| omega | Spin vector | rad/s |
| v | Linear velocity vector | m/s |
| k_m | Magnus coefficient | model coefficient |
| F_m | Magnus force | N-equivalent model force |

## Source and scope

Primary implementation:

- core/driver/include/frcsim/aerodynamics/magnus_model.hpp
- core/driver/include/frcsim/math/vector.hpp

Validation tests:

- vendordep/tests/magnus_test.cpp
- vendordep/tests/forces_test.cpp

JSim computes Magnus force direction from cross product:

$$
F_m = k_m (\omega \times v)
$$

where:

- omega: angular velocity vector (rad/s)
- v: linear velocity vector (m/s)
- k_m: Magnus coefficient

## Direction convention

Use right-hand rule for $$\omega \times v$$.

Example used by test:

- spin +Z
- velocity +X
- result +Y

So side force is positive Y in that frame.

Interpretation process:

1. Identify velocity and spin directions in the same frame.
2. Apply right-hand rule to omega x v.
3. Scale by k_m.

## Sign table quick check

- +Z x +X = +Y
- +X x +Y = +Z
- +Y x +Z = +X

If measured curve bends opposite expected side, either spin sign or frame transform is wrong.

## Units and coefficient meaning

k_m sets force magnitude scale and is tuned empirically in this implementation.

Because Magnus implementations vary across simulators, treat k_m as an identified parameter rather than a universal constant.

Practical behavior:

- larger |omega| increases side force
- larger |v| increases side force
- changing spin direction flips side force direction

## Interaction with drag and gravity

Trajectory acceleration combines effects:

$$
a = g + \frac{F_d}{m} + \frac{F_m}{m}
$$

Magnus mainly deflects path laterally while drag reduces speed and gravity drives vertical drop.

In practice, trajectory curvature depends on all three terms and their time-varying coupling through speed.

## Validation recipe

1. Set velocity along +X.
2. Set spin along +Z and verify +Y force.
3. Flip spin to -Z and verify -Y force.
4. Double spin and confirm force magnitude roughly doubles.
5. Disable drag to isolate Magnus-only behavior.

## Common mistakes

- Using v x omega instead of omega x v.
- Mixing world and body spin vectors.
- Tuning k_m before confirming sign and frame correctness.

## Validation in JSim

- vendordep/tests/magnus_test.cpp checks canonical omega x v sign
- vendordep/tests/forces_test.cpp checks integration with broader force pipeline

## Related Pages

- [Aerodynamics Math](aerodynamics.md)
- [Vector3](vector.md)
- [Physics Reference](physics_reference.md)
- [Units and Conventions](units_and_conventions.md)
