# JSim Vendordep

This package exposes JSim physics as a WPILib vendordep with both Java and C++ entry points.

## Implemented API Surface

### Java

Use `jsim.PhysicsWorld` and `jsim.PhysicsBody` for rigid-body simulation:

```java
import jsim.PhysicsBody;
import jsim.PhysicsWorld;
import jsim.Vec3;

try (PhysicsWorld world = new PhysicsWorld(0.01, true)) {
  PhysicsBody body = world.createBody(1.0);
  body.setPosition(new Vec3(0.0, 0.0, 1.0));
  body.setLinearVelocity(new Vec3(3.0, 0.0, 2.0));

  for (int i = 0; i < 100; ++i) {
    world.step();
  }

  Vec3 position = body.position();
}
```

Current Java API includes:
- Create/destroy worlds
- Create bodies
- Set body position and linear velocity
- Enable/disable per-body gravity
- Set global gravity vector
- Step simulation by N steps
- Read body position/velocity

### Built-in Field Definitions (No Python Runtime Step)

Use vendordep-bundled field JSON by year directly in Java runtime:

```java
import com.fasterxml.jackson.databind.JsonNode;
import jsim.field.FieldDefinitionCatalog;

JsonNode field = FieldDefinitionCatalog.loadFieldNode(2024);
// Use field geometry/elements in runtime setup.
```

This avoids requiring teams/students to run external Python scripts just to
load or move field elements.

### C++

Use `jsim::PhysicsWorld` from `src/main/driver/include/driverheader.h`.

## Build Notes

- Native and Java builds/tests are driven by vendordep Gradle tasks.
- Vendordep Gradle build requires a WPILib-compatible JDK (Java 21 recommended).

## Release Packaging

`JSim.json` is configured to publish Java, JNI driver, and C++ artifacts.
Before prerelease publishing:

1. Set desired version in `publish.gradle` (`pubVersion`).
2. Run `./gradlew test` in this folder on a supported JDK.
3. Validate artifacts in `build/outputs` and `build/repos`.
