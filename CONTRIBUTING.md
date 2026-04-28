# Contributing Guide

## Overview

This repository is a multi-language robotics simulation platform designed for FRC-style systems. It combines a native C++ physics engine, a Java simulation API, Python tooling, and runtime/visualization applications. The system is structured as a layered architecture where each layer has a strict responsibility boundary.

The goal of this document is to define those boundaries, explain how components interact, and establish rules for contributing to the codebase in a consistent and maintainable way.

---

## Architectural Philosophy

The repository follows a strict layered architecture:

```
Applications (apps, examples)
        ↑
Java Simulation API (jsim)
        ↑
JNI Bridge (core JNI layer)
        ↑
C++ Physics Engine (core)
        ↑
Mathematical and Physical Primitives
```

Each layer depends only on the layer directly beneath it. Lower layers must never depend on higher layers.

Supporting systems (field pipeline, CAD import, documentation, build scripts) exist parallel to this stack and must not introduce circular dependencies.

---

## Core Principles

### 1. Separation of Concerns

Each subsystem has a single responsibility:

* **core/**: Deterministic physics simulation and math engine (C++)
* **jsim/**: Java-facing simulation API and robotics abstraction layer
* **JNI layer**: Language interop only; no business logic
* **apps/**: Runtime execution, visualization, and tooling
* **cad-import/**: Geometry and field data processing pipeline
* **examples/**: Minimal reproducible usage demonstrations

No subsystem should assume implementation details of another layer beyond its direct dependency contract.

---

### 2. Dependency Direction Rules

Dependencies must follow this hierarchy strictly:

* Java API may depend on JNI bindings
* JNI may depend on C++ core
* Core must not depend on Java, Python, or application code
* Applications may depend on Java API only

Violations of dependency direction are considered architectural defects.

---

### 3. Physics Engine Integrity

The C++ core is the authoritative simulation system.

Rules:

* All physics computations must be deterministic or explicitly documented otherwise
* No UI, logging, or application logic is permitted in core
* No Java or JNI headers may be included in core logic files
* All state changes must originate from controlled simulation updates

---

### 4. Java API Design Rules

The Java layer serves as the user-facing robotics simulation API.

Rules:

* Must remain free of direct physics implementation logic
* Must interact with simulation state only through JNI interfaces
* Must provide stable abstractions for robot state, field state, and game pieces
* Duplicate state management logic across packages is prohibited

---

### 5. JNI Bridge Constraints

The JNI layer is strictly a translation boundary.

Rules:

* No simulation logic may exist in JNI code
* JNI functions must be thin wrappers around core C++ calls
* Data structures passed across the boundary must be minimal and well-defined
* Memory ownership must be explicitly documented for all cross-language objects

---

### 6. Field and Game Definitions

Field configuration and game-specific logic are data-driven.

Rules:

* Field definitions must be stored as versioned data files (JSON or equivalent)
* Field definitions must not embed executable logic unless explicitly required for parsing
* Multiple field versions (e.g., seasonal FRC changes) must remain isolated and backward-compatible
* Import/export tools must not modify core simulation behavior

---

### 7. Applications Layer (apps/)

The applications layer includes runtime and visualization tools.

Rules:

* Must not implement physics or simulation logic
* Must consume only public API interfaces
* May implement rendering, visualization, debugging tools, and orchestration logic
* Should be considered replaceable without affecting core correctness

---

### 8. Examples Layer

The examples directory serves as documentation through code.

Rules:

* Must be minimal and self-contained
* Must not introduce hidden dependencies or external configuration
* Must reflect intended usage of the public API

---

### 9. Build Artifacts and Generated Files

The repository must not commit generated build outputs.

Prohibited from version control:

* build/
* out/
* bin/
* .gradle/
* compiled class files

All generated artifacts must be reproducible via build scripts.

---

## Coding Standards

### C++ Core

* Prefer explicit memory ownership semantics
* Avoid global mutable state
* Use RAII for resource management
* Keep headers minimal and focused

### Java API

* Use clear, immutable data structures where possible
* Avoid leaking JNI implementation details
* Maintain backward-compatible interfaces when possible

### Python Tools

* Used primarily for data processing, import/export, and tooling
* Must not be required for core simulation execution

---

## Testing Requirements

All contributions must include appropriate tests where applicable:

* Core physics changes require C++ unit tests
* Java API changes require Java unit tests
* Field pipeline changes require validation against sample datasets

Tests must be deterministic and runnable in CI.

---

## Pull Request Guidelines

Each pull request must:

1. Clearly state which layer(s) are affected
2. Avoid cross-layer logic violations
3. Include tests or validation steps when modifying core behavior
4. Not introduce duplicate state systems or redundant abstractions
5. Preserve API compatibility unless explicitly breaking changes are justified

---

## Directory Ownership

* `core/` → physics engine maintainers
* `jsim/` → API maintainers
* `apps/` → tooling and visualization maintainers
* `cad-import/` → field/data pipeline maintainers
* `examples/` → documentation contributors
* `vendordep/` → build/integration maintainers

Cross-layer changes require coordination between owners.

---

## Design Intent Summary

The system is designed to separate:

* Physical simulation correctness (C++)
* User-facing robotics abstraction (Java)
* Execution and visualization (apps)
* Data-driven field configuration (pipeline tools)

Maintaining these separations is critical to long-term stability and usability of the platform.
