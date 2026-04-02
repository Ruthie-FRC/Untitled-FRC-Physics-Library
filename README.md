# RenSim Physics Library

[![CI](https://github.com/Ruthie-FRC/RenSim/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/Ruthie-FRC/RenSim/actions/workflows/ci.yml)
[![Deploy to rensim.dev](https://github.com/Ruthie-FRC/RenSim/actions/workflows/deploy-mkdocs.yml/badge.svg?branch=main)](https://github.com/Ruthie-FRC/RenSim/actions/workflows/deploy-mkdocs.yml)
[![License](https://img.shields.io/github/license/Ruthie-FRC/RenSim)](LICENSE.txt)
[![Last Commit](https://img.shields.io/github/last-commit/Ruthie-FRC/RenSim/main)](https://github.com/Ruthie-FRC/RenSim/commits/main)
[![Latest Release](https://img.shields.io/github/v/release/Ruthie-FRC/RenSim)](https://github.com/Ruthie-FRC/RenSim/releases)
[![Open Issues](https://img.shields.io/github/issues/Ruthie-FRC/RenSim)](https://github.com/Ruthie-FRC/RenSim/issues)
[![GitHub Stars](https://img.shields.io/github/stars/Ruthie-FRC/RenSim?style=social)](https://github.com/Ruthie-FRC/RenSim/stargazers)

RenSim is a modular FRC physics library for simulation, analysis, and robotics workflow integration.

## What Is In This Repository

This is a monorepo with core physics code, runtime apps, examples, and documentation tooling.

- `core/`
	- `core/driver/`: C++ physics engine implementation and headers
	- `core/java/`: Java-side code and bindings
	- `core/python/`: Python-side code and bindings
	- `core/bindings-java/`: Java binding support
	- `core/gamepiece-models/`: gamepiece model definitions
	- `core/physics-core/`: shared physics core components
- `apps/`
	- `apps/sim-runtime/`: Python runtime integration app
	- `apps/viewer-plugin/`: visualization/rendering plugin
- `cad-import/`: CAD and geometry import utilities
- `examples/`: language-specific usage examples
- `mkdocs/`: docs sources and MkDocs config (`mkdocs/docs/`, `mkdocs/mkdocs.yml`)
- `vendordep/`: Gradle/WPILib vendordep packaging and tests
- `physics-core/`: additional physics-core headers/source tree

## Build (C++)

This repository is configured with CMake.

```bash
cmake -S . -B build
cmake --build build -j
```

## Run Tests

Primary CI tests are run via the helper script:

```bash
./scripts/run-tests.sh
```

You can pass through extra Gradle flags:

```bash
./scripts/run-tests.sh --info
```

If you also want to run CTest-based native tests from the CMake build:

```bash
ctest --test-dir build --output-on-failure
```

## Docs

Docs are built and deployed from the MkDocs config at `mkdocs/mkdocs.yml`.

Local docs preview:

```bash
pip install mkdocs mkdocs-material
mkdocs serve --config-file mkdocs/mkdocs.yml
```