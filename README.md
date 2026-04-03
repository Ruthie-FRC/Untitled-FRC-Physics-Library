# RenSim Physics Library

[![CI](https://github.com/Ruthie-FRC/RenSim/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/Ruthie-FRC/RenSim/actions/workflows/ci.yml)
[![Deploy to rensim.dev](https://github.com/Ruthie-FRC/RenSim/actions/workflows/deploy-mkdocs.yml/badge.svg?branch=main)](https://github.com/Ruthie-FRC/RenSim/actions/workflows/deploy-mkdocs.yml)
[![License](https://img.shields.io/github/license/Ruthie-FRC/RenSim)](LICENSE.txt)
[![Latest Release](https://img.shields.io/github/v/release/Ruthie-FRC/RenSim)](https://github.com/Ruthie-FRC/RenSim/releases)
[![Open Issues](https://img.shields.io/github/issues/Ruthie-FRC/RenSim)](https://github.com/Ruthie-FRC/RenSim/issues)

RenSim is a modular FRC physics library for simulation, analysis, and robotics workflow integration.

**Status**: 2026.04.03.0-prerelease — Core rigid-body dynamics and joint constraints.             

## Overview

This repository is organized as a monorepo containing the core simulation engine, language bindings, runtime apps, examples, and documentation tooling.

## Quick Start

### Build (C++)

```bash
cmake -S . -B build
cmake --build build -j
```

### Run Tests

Run the full test helper used in CI:

```bash
./scripts/run-tests.sh
```

Pass additional Gradle flags when needed:

```bash
./scripts/run-tests.sh --info
```

Run native CTest suites from the CMake build directory:

```bash
ctest --test-dir build --output-on-failure
```

### Preview Docs Locally

```bash
pip install mkdocs mkdocs-material
mkdocs serve --config-file mkdocs/mkdocs.yml
```

## Repository Structure

### Core Libraries

- `core/driver/`: C++ physics engine implementation and headers
- `core/physics-core/`: shared physics components
- `core/java/`: Java-side code and bindings
- `core/python/`: Python-side code and bindings
- `core/bindings-java/`: Java binding support
- `core/gamepiece-models/`: gamepiece model definitions
- `physics-core/`: additional physics core include/source tree

### Applications

- `apps/sim-runtime/`: Python runtime integration app
- `apps/viewer-plugin/`: visualization and rendering plugin

### Tooling and Integration

- `cad-import/`: CAD and geometry import utilities
- `examples/`: language-specific examples (C++, Java, Python)
- `mkdocs/`: docs source and MkDocs configuration
- `vendordep/`: WPILib vendordep packaging and Gradle-based testing

## Documentation

- Docs source: `mkdocs/docs/`
- MkDocs config: `mkdocs/mkdocs.yml`
- Deployed docs: https://rensim.dev

## Contributing

Contributions are welcome. For code and process expectations, please review:

- `CODE_OF_CONDUCT.md`
- open issues and discussions in this repository

## License

This project is licensed under the terms in `LICENSE.txt`.
