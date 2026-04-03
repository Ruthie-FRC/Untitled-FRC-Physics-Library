# Architecture

RenSim is organized as a modular monorepo with clear separation between core physics, bindings, runtime applications, and documentation.

## High-Level Structure

- `core/driver/`: Primary C++ engine headers and implementation.
- `core/physics-core/`: Shared C++ physics modules.
- `core/python/`: Python-side bindings and integration code.
- `core/java/`: Java-side bindings and API surface.
- `core/bindings-java/`: Java/JNI binding support.
- `core/gamepiece-models/`: Gamepiece and simulation object models.
- `physics-core/`: Additional top-level physics include/source tree.
- `apps/sim-runtime/`: Runtime integration for robot simulation workflows.
- `apps/viewer-plugin/`: Visualization and timeline tooling.
- `cad-import/`: CAD/geometry import utilities (URDF/STL processing).
- `examples/`: Language-specific examples.
- `scripts/`: Project scripts including test orchestration.
- `vendordep/tests/`: Native unit and integration test sources.

## Physics Core Design

Key components include:

- Math primitives: vectors, quaternions, matrices, and integrators.
- Rigid-body modeling and assemblies.
- Force generators and aerodynamic models.
- Physics world orchestration.

The architecture is designed to support extension through additional force models, body types, and runtime integrations.

## Build and Test

- Gradle drives native, JNI, and Java builds.
- `vendordep/gradlew test` executes Java tests and native verification binaries.
- `scripts/run-tests.sh` enforces Java 21 and runs the vendordep test workflow from repo root.

## Documentation Strategy

Documentation is authored in Markdown under `mkdocs/docs/` and published with MkDocs Material.

## Related Pages

- [API Usage](api_usage.md)
- [Physics Reference](physics_reference.md)
