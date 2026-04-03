# API Usage

This page describes the primary usage patterns for RenSim in C++, Python, and Java.

## C++

Use the C++ examples as the reference entry point:

```cpp
// See examples/cpp/minimal_world.cpp for a complete runnable setup.
```

Typical flow:

- Create a physics world.
- Add rigid bodies and configure mass/inertia.
- Apply forces (gravity, springs, motors, or custom force generators).
- Step the world using a fixed timestep.

## Python

```bash
python examples/python/simple_world_demo.py
```

Typical flow:

- Import bindings/runtime modules and create a simulation world.
- Configure bodies and environment.
- Step simulation in a loop and read back state.

## Java

Use the Java example as the reference entry point:

```java
// See examples/java/ShooterPredictionExample.java for usage.
```

Typical flow:

- Create physics objects through Java bindings.
- Configure system parameters.
- Step simulation and consume state for robot logic.

## Best Practices

- Use a fixed timestep for deterministic behavior.
- Keep units consistent (SI recommended).
- Validate with tests before tuning coefficients.
- Prefer explicit force models over ad-hoc state edits.

## Related Pages

- [Architecture](architecture.md)
- [Physics Reference](physics_reference.md)
- [Integrators](integrators.md)
