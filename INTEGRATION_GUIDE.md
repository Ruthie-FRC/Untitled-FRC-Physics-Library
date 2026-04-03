# RenSim Integration Guide for FRC Teams

## Adding RenSim to Your FRC Project

### Step 1: Add Vendordep to WPILib

In VS Code with the WPILib extension:
1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`)
2. Run: **WPILib: Manage Vendor Libraries**
3. Select: **Install new libraries (offline)**
4. Point to: `RenSim.json` from the RenSim repository release

Alternatively, manually copy `RenSim.json` to `vendordeps/` folder in your robot project.

### Step 2: Configure Your Robot Project

In **build.gradle**:
```gradle
dependencies {
    // Your existing WPILib deps...
    
    // Add RenSim
    nativeDesktopZip wpilib.deps.vendor.rensim()
    nativeDesktopZip wpilib.deps.vendor.rensim.driver()
}
```

### Step 3: Use Physics in Your Code

#### Example 1: Simulate a Shooter

```java
import com.vendor.physics.PhysicsWorld;
import com.vendor.physics.PhysicsBody;
import com.vendor.physics.Vec3;

public class ShooterSimulator {
    private PhysicsWorld physicsWorld;
    private PhysicsBody shooterWheel;
    private PhysicsBody gamepiece;
    
    public ShooterSimulator() {
        physicsWorld = new PhysicsWorld(0.0025, true);  // 400 Hz simulation
        
        // Wheel (spinning mass)
        shooterWheel = physicsWorld.createBody(0.5);  // 500g wheel
        shooterWheel.setGravityEnabled(false);  // Don't fall
        
        // Game piece
        gamepiece = physicsWorld.createBody(0.235);  // ~235g power cells
    }
    
    public Vec3 predictTrajectory(double launchVelocityMps, double angle) {
        // Fire game piece from (0,0,1) at angle
        double vx = launchVelocityMps * Math.cos(Math.toRadians(angle));
        double vz = launchVelocityMps * Math.sin(Math.toRadians(angle));
        
        gamepiece.setPosition(new Vec3(0.0, 0.0, 1.0));
        gamepiece.setLinearVelocity(new Vec3(vx, 0.0, vz));
        
        Vec3 maxHeight = null;
        double maxZ = 1.0;
        
        for (int i = 0; i < 500; ++i) {
            physicsWorld.step();
            Vec3 pos = gamepiece.position();
            
            if (pos.z() > maxZ) {
                maxZ = pos.z();
                maxHeight = pos;
            }
            
            if (pos.z() <= 0.0) {
                return pos;  // Hit ground
            }
        }
        
        return gamepiece.position();
    }
    
    public void shutdown() {
        physicsWorld.close();
    }
}
```

#### Example 2: Multi-Body Arm

```java
// For linked arm segments with revolute joints (v0.2+)
// Once collision detection is enabled, you can validate arm poses
```

### Step 4: Run Simulations in Tests

```java
import org.junit.jupiter.api.Test;

public class ShooterSimulatorTest {
    @Test
    void testTrajectoryRange() {
        try (var sim = new ShooterSimulator()) {
            Vec3 impact = sim.predictTrajectory(15.0, 45.0);
            
            double range = Math.sqrt(impact.x() * impact.x() + impact.y() * impact.y());
            System.out.printf("Shot at 45°: range=%.2f m%n", range);
            
            // Add assertions based on your field dimensions
        }
    }
}
```

## API Reference

### PhysicsWorld (Main Entry Point)

```java
// Create world with time step (seconds) and gravity enabled
PhysicsWorld world = new PhysicsWorld(0.01, true);

// Create rigid bodies
PhysicsBody body = world.createBody(double massKg);

// Control simulation
world.step();           // Step by 1 timestep
world.step(10);         // Step by N timesteps
world.setGravity(new Vec3(0, 0, -9.81));

// Resource management
world.close();          // Cleanup native resources (or use try-with-resources)
```

### PhysicsBody

```java
// Pose
body.setPosition(new Vec3(x, y, z));
Vec3 pos = body.position();

// Velocity
body.setLinearVelocity(new Vec3(vx, vy, vz));
Vec3 vel = body.linearVelocity();

// Flags
body.setGravityEnabled(true);

// Future: Rotation, angular velocity, forces (v0.2+)
```

### Vec3 (3D Vector)

```java
Vec3 v = new Vec3(x, y, z);
double x = v.x();
double y = v.y();
double z = v.z();

Vec3 ZERO = Vec3.ZERO;

// Arithmetic operators coming in v0.2
```

## Simulation Best Practices

1. **Choose appropriate time step**: 
   - `0.005` (200 Hz) for accuracy
   - `0.01` (100 Hz) for balance
   - `0.02` (50 Hz) for coarse estimates

2. **Pre-allocate bodies**: Create all bodies upfront; avoid dynamic creation in loops.

3. **Use AutoCloseable**: Always close PhysicsWorld to free native resources:
   ```java
   try (PhysicsWorld w = new PhysicsWorld(0.01, true)) {
       // Your code
   }  // Automatically closed
   ```

4. **Validate against reality**: Tune material coefficients and forces against real robot tests.

5. **Cache PhysicsWorld**: Create once, reuse across multiple predictions. Don't recreate per call.

## Troubleshooting

### "UnsatisfiedLinkError: no VendorDriver in java.library.path"
- Ensure RenSim.json is properly added via vendordep manager
- Check that the JNI driver library built successfully for your platform
- Run `./gradlew build` in the RenSim vendordep folder to rebuild artifacts

### Unrealistic Behavior
- Check gravity vector: default is (0, 0, -9.81) m/s²
- Verify mass values are in kg
- Ensure velocity units are m/s
- Check for extremely high forces (can cause numerical instability)

### Performance / Lag
- Reduce step count per iteration: `world.step()` instead of `world.step(100)`
- Increase time step (less accuracy but faster)
- Profile using a profiler to identify bottlenecks

## Next Steps

- **v0.2 (Q3 2026)**: Collision detection, improved boundaries, assembly utilities
- **v0.3 (Q4 2026)**: Soft-body deformation, CAD import integration
- **v1.0 (2027)**: Real-time visualization, multibody optimization, full FRC integration

---

## Questions?

Post issues on the [RenSim GitHub repository](https://github.com/Ruthie-FRC/RenSim/issues).
