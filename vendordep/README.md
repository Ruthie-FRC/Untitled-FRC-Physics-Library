# RenSim Vendordep

This package exposes RenSim physics as a WPILib vendordep with both Java and C++ entry points.

## Implemented API Surface

### Java

Use `com.vendor.physics.PhysicsWorld` and `com.vendor.physics.PhysicsBody` for rigid-body simulation:

```java
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

### C++

Use `rensim::PhysicsWorld` from `src/main/native/include/header.h`.

## Build Notes

- Native and Java builds/tests are driven by vendordep Gradle tasks.
- Vendordep Gradle build requires a WPILib-compatible JDK (Java 21 recommended).

## Release Packaging

`RenSim.json` is configured to publish Java, JNI driver, and C++ artifacts.
Before prerelease publishing:

1. Set desired version in `publish.gradle` (`pubVersion`).
2. Run `./gradlew test` in this folder on a supported JDK.
3. Validate artifacts in `build/outputs` and `build/repos`.
