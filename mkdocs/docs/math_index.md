# Math Overview

This section documents the mathematical foundations of JSim and connects each model to implementation and tests.

## Documentation goals

- Explain not only what equations are used, but why they are used.
- Make frame conventions, units, and sign rules explicit.
- Provide practical validation steps that can be run in this repository.

## Recommended reading order

1. Units and Conventions
2. Vector3
3. Matrix3
4. Quaternion
5. Integrators
6. Aerodynamics Math
7. Magnus Effect Math
8. Collision Impulses
9. Drivetrain and Mechanism Dynamics

## How to use these pages effectively

1. Identify the model and write down the governing equation.
2. Confirm symbol meaning and units from the symbol table.
3. Check frame consistency (body vs world) before tuning constants.
4. Validate signs and magnitudes using linked tests.
5. Tune coefficients only after the above checks pass.

## Test map

- vendordep/tests/math_test.cpp: vector, quaternion, matrix fundamentals
- vendordep/tests/integration_test.cpp: linear/angular integration behavior
- vendordep/tests/forces_test.cpp: drag, gravity, geometry-dependent area, force accumulation
- vendordep/tests/magnus_test.cpp: Magnus force direction convention
- vendordep/tests/boundary_test.cpp: boundary data model and integration plumbing
