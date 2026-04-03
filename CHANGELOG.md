# Changelog

All notable changes to RenSim are documented in this file.

## Versioning

RenSim uses date-based versioning: `YYYY.MM.DD.patch-no`
- `patch-no` increments for multiple releases on the same day
- Format: `2026.04.03.0` is the first release on April 3, 2026
- Prerelease versions append `-prerelease` suffix

## [2026.04.03.0-prerelease] - 2026-04-03

### Added
- **Rigid Body Physics**: Complete rigid-body simulation with mass, inertia, linear/angular velocity, and force/torque application
- **Joint Constraints**: 
  - Fixed joints for rigid connections
  - Revolute joints with optional angle limits and motor control
  - Prismatic joints with optional displacement limits and motor control
- **Integration Methods**: Semi-implicit Euler, explicit Euler, Runge-Kutta 2nd-order
- **Force Generators**: 
  - Gravity (configurable per-body and global)
  - Motor forces (from torque/force targets)
  - Spring forces
- **Aerodynamics**: 
  - Drag force modeling with quadratic and linear terms
  - Magnus force (spin-induced lift)
  - Spin decay
- **Assemblies**: Multi-body systems with constraint solving
- **Material System**: Restitution, kinetic/static friction, collision damping
- **Java/JNI API**: Complete vendordep with PhysicsWorld, PhysicsBody, Vec3
- **Examples**: Java shooter trajectory prediction, C++ minimal world simulations
- **Comprehensive Test Suite**: 9 tests covering all major systems

### Fixed
- Drag force diagnostic struct memory layout for header self-containment
- Constraint solver impulse calculations for joint stability
- JNI symbol export for vendordep driver

### Known Issues / Planned Features
- Collision detection: Placeholder structure, not yet implemented (v0.2)
- Soft-body deformation: Framework scaffolded, not wired (v0.2)
- Field boundaries: Stub implementations (v0.2)
- CAD import: Utilities present but not integrated (v0.3)

### Platform Support
- Linux (x86-64, ARM64)
- macOS (Universal)
- Windows (x86-64, ARM64)
- RoboRIO (via WPILib vendordep)

### API Stability
This is a **prerelease**. Core physics API (PhysicsWorld, RigidBody) is stable. Joint configuration and constraint parameters may refine in v0.2 based on FRC team feedback.

---

## Development Notes

### Architecture Overview
- **core/driver**: Main C++ physics engine with constraint solvers
- **core/physics-core**: Alternative physics API entry point (legacy, not actively maintained in prerelease)
- **vendordep**: WPILib wrapper, JNI bindings, Java API
- **apps**: Device simulators and runtime integration
- **examples**: Usage demonstrations for C++ and Java

### Build System
- **Gradle 8.14.3**: For native C++, JNI, and Java builds/tests

### Testing
Run `cd vendordep && ./gradlew test` to validate:
- Core physics algorithms
- Integration methods
- Joint constraint solving
- Force application
- Aerodynamic modeling
