# Jsim 2026.04.03.0-prerelease Quick Start

Jsim is a physics simulation framework for FRC-style robotics. This prerelease (2026.04.03.0) includes a fully functional rigid-body dynamics engine with joint constraints, aerodynamic forces, and a complete Java API for FRC teams.

## What's Included

### Core Physics Engine (C++)
- **Rigid Body Dynamics**: Mass, inertia, linear/angular velocity, forces, torque
- **Integration Methods**: Semi-implicit Euler, explicit Euler, RK2
- **Joint Constraints**:
  - Fixed joints (rigid connections)
  - Revolute joints (hinges with optional angle limits and motors)
  - Prismatic joints (linear sliders with optional displacement limits and motors)
- **Force Generators**: Gravity, motors, springs
- **Aerodynamics**: Magnus force, drag models, spin decay
- **Assemblies**: Connect multiple bodies with constraints in a single unit
- **Material Properties**: Restitution, friction coefficients

### Java/JNI Bindings
Complete vendordep package with:
- `PhysicsWorld`: Create and step simulations
- `PhysicsBody`: Query and control body state
- `Vec3`: 3D vector type

### Examples
- [ShooterPredictionExample.java](examples/java/ShooterPredictionExample.java): Projectile trajectory with gravity

## Building

### Core Build and Native Tests
```bash
./scripts/build-all.sh
```

This script is safe to run from any working directory and handles both native and vendordep builds.

### Vendordep (Java)
Requires Java 21:
```bash
cd vendordep
./gradlew build
```

## Java Usage Example

```java
import rensim.PhysicsWorld;
import rensim.PhysicsBody;
import rensim.Vec3;

try (PhysicsWorld world = new PhysicsWorld(0.01, true)) {
    // Create a 1 kg body at (0, 0, 1) m
    PhysicsBody body = world.createBody(1.0);
    body.setPosition(new Vec3(0.0, 0.0, 1.0));
    body.setLinearVelocity(new Vec3(3.0, 0.0, 2.0));

    // Simulate 100 steps (1 second at 100 Hz)
    for (int i = 0; i < 100; ++i) {
        world.step();
    }

    // Read final state
    Vec3 pos = body.position();
    Vec3 vel = body.linearVelocity();
    System.out.printf("Final: pos=%.2f,%.2f,%.2f vel=%.2f,%.2f,%.2f%n",
        pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z());
}
```

## C++ Usage Example

```cpp
#include "header.h"
#include "frcsim/physics_world.hpp"

frcsim::PhysicsConfig config;
config.fixed_dt_s = 0.01;
config.enable_gravity = true;

frcsim::PhysicsWorld world(config);
frcsim::RigidBody& body = world.createBody(1.0);
body.setPosition(frcsim::Vector3(0.0, 0.0, 1.0));

for (int i = 0; i < 100; ++i) {
    world.step();
}

std::cout << "Position: " << body.position() << std::endl;
```

## Known Limitations

This v0.1.0-prerelease focuses on rigid-body dynamics and joint constraints:
- **Collision detection**: Disabled (planned for v0.2)
- **Soft body deformation**: Framework exists but not implemented (planned for v0.2)
- **Field boundaries**: Structure exists but logic not wired (planned for v0.2)
- **CAD import**: Utilities exist but not integrated (planned for v0.2)

## Test Coverage

All 9 core tests pass:
- Integration methods and gravity
- Rigid body dynamics
- Rigid assemblies with joint constraints
- Force generators and damping
- Aerodynamics and Magnus effects
- Material properties
- Boundary structures
- Deformable body scaffolding

Run `cd vendordep && ./gradlew test` to validate your environment.

## Prerelease Verification

Run the full prerelease gate from repository root:

```bash
cd vendordep
./gradlew test
```

Expected verification signals:
- Gradle reports `BUILD SUCCESSFUL`.
- The test command prints `Running legacy native test:` entries for all standalone native suites in `vendordep/tests/`.
- No assertion failures or non-zero exits occur while running legacy native tests.

## Prerelease Notes

This release provides a **solid foundation for physics-based simulations**. The constraint solver and force integration are production-ready for:
- Trajectory prediction (shooters, intakes, climbers)
- Mechanism tuning (arm kinematics, drivetrain dynamics)
- Control algorithm validation

For FRC teams: Integrate via WPILib vendordep manager. Add `JSim.json` to your project's vendordep directory and reference via imports.

## Next Steps

- v0.2: Collision detection, improved boundary constraints
- v0.3: Soft-body deformation, expanded CAD import
- v1.0: Full FRC integration, real-time visualization, multibody optimization

## Support

Issues, questions, or feature requests? File them on the project repository.
