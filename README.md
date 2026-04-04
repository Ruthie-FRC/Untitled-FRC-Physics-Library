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

### Build and Test (Gradle)

```bash
./scripts/build-all.sh
```

For vendordep-only builds:

```bash
cd vendordep
./gradlew test
```

### Run Tests

Run the vendordep Gradle test task used in CI:

```bash
cd vendordep
./gradlew test
```

This command runs Java tests plus the standalone native verification suite in `vendordep/tests/` through Gradle.

Vendordep build tooling currently requires Java 21.

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

### Integration Contracts

- Runtime telemetry contract: `apps/sim-runtime/telemetry_schema.py`
- Java telemetry IO/parsing: `core/java/rensim/simulation/telemetry/`
- Shared NT-style flattened key layout: `sim/tick`, `sim/time_s`, `sim/contact_count`, `sim/body/{i}/*`

### Tooling and Integration

- `cad-import/`: CAD and geometry import utilities
- `examples/`: language-specific examples (C++, Java, Python)
- `mkdocs/`: docs source and MkDocs configuration
- `vendordep/`: WPILib vendordep packaging and Gradle-based build/testing

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
