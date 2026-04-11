# JSim
JSim is a modular FRC physics library for simulation, analysis, and robotics workflow integration.

!!! info "Project Status"
    JSim is under active development. Core simulation features and additional capabilities are being added iteratively.

## Why JSim

- Unified physics engine for rigid-body, drivetrain, and mechanism simulation
- Multi-language support with native C++ and Python/Java bindings
- Simulation runtime components for robot code and sensor pipelines
- CAD import workflows for URDF, STL, and related geometry formats
- Modular architecture for custom models, forces, and plugins

## Project Goals

- Accuracy: physically grounded simulation for FRC robots and game elements
- Integration: seamless workflow from CAD to code to simulation
- Accessibility: practical for teams of varied experience levels
- Open source: community-driven and MIT-licensed

## Quick Start

### Build and Test

```bash
scripts/run-tests.sh
```

### Direct Gradle Workflow

```bash
cd vendordep
./gradlew test
```

`scripts/run-tests.sh` is the recommended path from repository root and expects Java 21.

### Run Examples

- C++: `examples/cpp/minimal_world.cpp`
- Python: `examples/python/simple_world_demo.py`
- Java: `examples/java/ShooterPredictionExample.java`


## Documentation Map

- [API Usage](api_usage.md): integrating JSim into applications
- [Java API Reference (Javadocs)](/api/javadoc/): generated Java bindings and APIs
- [Native API Reference (Doxygen)](/api/doxygen/html/): generated C++ and mixed-language symbols
- [Architecture](architecture.md): subsystem layout and extension model
- [Physics Reference](physics_reference.md): models, assumptions, and equations
- [Math Overview](math_index.md): math modules and conventions
- [Integrators](integrators.md): numerical integration methods
- [Vector3](vector.md), [Matrix3](matrix.md), [Quaternion](quaternion.md): core math primitives
- [Units and Conventions](units_and_conventions.md): coordinate and unit standards

## Community and Contribution

Contributions are welcome from teams, mentors, and developers across the FRC ecosystem.

- Report bugs and request features in [GitHub Issues](https://github.com/Ruthie-FRC/JSim/issues)
- Propose improvements through pull requests
- Follow project standards in the repository documentation


<!--
<div align="center">
  <img src="assets/images/frc_field.png" alt="FRC Field" width="400"/>
</div>
-->
