# Quaternion (frcsim::Quaternion)

This page explains quaternion math used by RenSim for stable 3D orientation updates.

## Source and scope

Primary implementation:

- core/driver/include/frcsim/math/quaternion.hpp
- core/driver/include/frcsim/math/integrators.hpp

Validation tests:

- vendordep/tests/math_test.cpp
- vendordep/tests/integration_test.cpp

## Representation

RenSim stores quaternion as:

$$
q = (w, x, y, z) = (w, \mathbf{v})
$$

Identity rotation is:

$$
q_I = (1,0,0,0)
$$

## Hamilton product used by implementation

For two quaternions $$q_1 = (w_1, x_1, y_1, z_1)$$ and $$q_2 = (w_2, x_2, y_2, z_2)$$:

$$
q_1 q_2 =
\left(
w_1w_2 - x_1x_2 - y_1y_2 - z_1z_2,
\ w_1x_2 + x_1w_2 + y_1z_2 - z_1y_2,
\ w_1y_2 - x_1z_2 + y_1w_2 + z_1x_2,
\ w_1z_2 + x_1y_2 - y_1x_2 + z_1w_2
\right)
$$

This product is non-commutative.

## Vector rotation

Embed vector $$v$$ as pure quaternion $$p=(0,v)$$ and rotate with:

$$
p' = q p q^{-1}
$$

The rotated vector is the xyz part of $$p'$$.

RenSim rotate helper uses conjugate for unit quaternions, which is efficient and stable when normalization is maintained.

## Axis-angle conversion

From axis u and angle theta:

$$
q = \left(\cos\frac{\theta}{2},\ u\sin\frac{\theta}{2}\right)
$$

Near zero angle, axis is not uniquely defined; implementation falls back to a default axis when needed.

## Why q and -q are equivalent

Both represent the same 3D rotation because they map to the same rotation matrix and produce identical rotated vectors.

Implication for interpolation:

- flip one endpoint sign when dot product is negative to take shortest SLERP path

## SLERP behavior

For interpolation parameter t in [0,1], SLERP follows the geodesic on unit quaternion sphere.

RenSim behavior:

- flips sign when needed for shortest path
- falls back to normalized lerp when quaternions are very close

## Quaternion integration and normalization

RenSim angular integration uses:

$$
\dot{q} = \frac{1}{2}\omega_q q
$$

with forward step and normalize-if-needed.

Why normalization frequency matters:

- too infrequent: drift accumulates and rotation quality degrades
- every step: robust and simple for real-time simulation

## Worked 90-degree example

Rotate vector $$v=(1,0,0)$$ by +90 degrees about Z.

Axis-angle quaternion:

$$
q = \left(\cos\frac{\pi}{4}, 0,0,\sin\frac{\pi}{4}\right)
$$

Applying $$v' = q v q^{-1}$$ yields approximately:

$$
v' = (0,1,0)
$$

## Common mistakes

- using non-unit quaternions for repeated rotation steps
- multiplying in wrong order when composing rotations
- mixing body and world angular velocity definitions
- forgetting that q and -q are equivalent during comparisons

## Related Pages

- [Units and Conventions](units_and_conventions.md)
- [Matrix3](matrix.md)
- [Integrators](integrators.md)
