# Architecture

RenSim is organized as a modular monorepo with clear separation between core physics, bindings, runtime applications, and documentation.

## High-Level Structure

- `core/physics-core/`: C++ physics engine implementation.
- `core/bindings-python/`: Python bindings.
- `core/bindings-java/`: Java bindings.
- `core/gamepiece-models/`: Gamepiece models for simulation.
- `apps/sim-runtime/`: Runtime integration for robot simulation workflows.
- `apps/viewer-plugin/`: Visualization and timeline tooling.
- `examples/`: Language-specific examples.
- `vendordep/tests/`: Unit and integration tests.

## Physics Core Design

Key components include:

- Math primitives: vectors, quaternions, matrices, and integrators.
- Rigid-body modeling and assemblies.
- Force generators and aerodynamic models.
- Physics world orchestration.

The architecture is designed to support extension through additional force models, body types, and runtime integrations.

## Build and Test

- CMake drives native builds.
- CTest executes test binaries.
- Helper scripts streamline configure/build/test workflows.

## Documentation Strategy

Documentation is authored in Markdown under `mkdocs/docs/` and published with MkDocs Material.

## Related Pages

- [API Usage](api_usage.md)
- [Physics Reference](physics_reference.md)
