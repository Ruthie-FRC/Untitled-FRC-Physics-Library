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

## Simulation Integration Pipeline

- Python runtime simulation produces typed telemetry packets in `apps/sim-runtime/telemetry_schema.py`.
- Graphics rendering consumes timeline frames via `apps/sim-runtime/graphics_bridge.py` and `apps/viewer-plugin/`.
- Java consumers parse the same JSONL telemetry format through `core/java/rensim/simulation/telemetry/SensorPacketIO.java`.
- Telemetry packets carry explicit frame tags per body: `position_frame_tag` and `velocity_frame_tag` with values `w` (world) or `b` (body).
- Telemetry API boundaries validate SI field names (`*_m`, `*_mps`, `*_s`) and finite/non-negative values before flatten/export.
- Flattened NetworkTables-style key layout is shared across Python and Java:
	- `sim/tick`, `sim/time_s`, `sim/contact_count`
	- `sim/body/{i}/x_m`, `sim/body/{i}/y_m`, `sim/body/{i}/vx_mps`, `sim/body/{i}/vy_mps`, `sim/body/{i}/speed_mps`

### Visualization Controls

- Runtime options expose graphics quality profiles (`low`, `medium`, `high`) and per-feature render toggles.
- Generated dashboard page includes a sidebar for quality and feature toggles during playback.
- Generated control-bindings page allows driver/co-driver button mapping reassignment via browser local storage.

## Physics Core Design

Key components include:

- Math primitives: vectors, quaternions, matrices, and integrators.
- Rigid-body modeling and assemblies.
- Force generators and aerodynamic models.
- Physics world orchestration.

The architecture is designed to support extension through additional force models, body types, and runtime integrations.

### Math Standard Alignment

- Java `rensim.Vec3` now mirrors core C++ `frcsim::Vector3` utility surfaces used by simulation math.
- Java `frcsim_physics.RigidBody.Quaternion` follows C++ `frcsim::Quaternion` multiplication and angular integration shape (`q_dot = 0.5 * omega_quat * q`).
- Parity tests in vendordep verify Java-side vector/quaternion behavior stays consistent with C++ math conventions.
- A shared fixture at `vendordep/src/test/resources/math/cpp_math_contract.json` acts as a C++ math contract consumed by both Java and Python validation checks.

## Build and Test

- Gradle drives native, JNI, and Java builds.
- `vendordep/gradlew test` executes Java tests and native verification binaries.
- `scripts/run-tests.sh` enforces Java 21 and runs the vendordep test workflow from repo root.

## Documentation Strategy

Documentation is authored in Markdown under `mkdocs/docs/` and published with MkDocs Material.

## Related Pages

- [API Usage](api_usage.md)
- [Physics Reference](physics_reference.md)
