# Getting Started

Use this section as the entry point for integrating JSim into your project.

## Vendordep URL

```
https://jsim.dev/JSim.json
```

## Build and Test

From the repository root:

```bash
scripts/run-tests.sh
```

This script selects a Java 21 runtime and runs the vendordep Gradle test workflow.

You can also run the tests directly:

```bash
cd vendordep
./gradlew test
```

## Local Documentation Preview

```bash
pip install mkdocs mkdocs-material
mkdocs serve --config-file mkdocs/mkdocs.yml
```

## Example Entry Points

- C++: `examples/cpp/minimal_world.cpp`
- Java: `examples/java/ShooterPredictionExample.java`
- Python: `examples/python/simple_world_demo.py`

## Repository Areas

- `core/`: engine and language binding sources
- `apps/`: runtime and visualization tools
- `cad-import/`: CAD and geometry import utilities
- `vendordep/`: Gradle build/test and vendordep packaging
- `mkdocs/`: documentation source and configuration

## Pages

- [API Usage](api_usage.md): how to use the library from your application
- [Architecture](architecture.md): subsystem layout and extension model
