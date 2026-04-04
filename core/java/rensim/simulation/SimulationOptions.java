package rensim.simulation;

import java.util.Objects;
import rensim.Vec3;

/**
 * Aggregated simulation configuration inspired by full-featured arena simulators.
 */
public final class SimulationOptions {
  /** Global timing options. */
  public record Timing(double fixedDtSeconds, int subTicksPerRobotPeriod) {
    public Timing {
      if (!(fixedDtSeconds > 0.0)) {
        throw new IllegalArgumentException("fixedDtSeconds must be > 0");
      }
      if (subTicksPerRobotPeriod < 1) {
        throw new IllegalArgumentException("subTicksPerRobotPeriod must be >= 1");
      }
    }
  }

  /** Global collision settings. */
  public record Collision(boolean enabled, double defaultRestitution, boolean publishContacts) {
    public Collision {
      if (defaultRestitution < 0.0 || defaultRestitution > 1.0) {
        throw new IllegalArgumentException("defaultRestitution must be in [0, 1]");
      }
    }
  }

  /** Boundary settings for a rectangular field. */
  public record Boundaries(double widthMeters, double heightMeters, boolean enableWalls) {
    public Boundaries {
      if (!(widthMeters > 0.0)) {
        throw new IllegalArgumentException("widthMeters must be > 0");
      }
      if (!(heightMeters > 0.0)) {
        throw new IllegalArgumentException("heightMeters must be > 0");
      }
    }
  }

  /** Aerodynamic options for game piece and projectile simulation. */
  public record Aerodynamics(boolean enableDrag, boolean enableMagnus, double dragCoefficient,
      double magnusCoefficient, double spinDecayPerSecond) {
    public Aerodynamics {
      if (dragCoefficient < 0.0) {
        throw new IllegalArgumentException("dragCoefficient must be >= 0");
      }
      if (magnusCoefficient < 0.0) {
        throw new IllegalArgumentException("magnusCoefficient must be >= 0");
      }
      if (spinDecayPerSecond < 0.0) {
        throw new IllegalArgumentException("spinDecayPerSecond must be >= 0");
      }
    }
  }

  /** Optional projectile behavior parameters. */
  public record Projectile(boolean enableTrajectoryPreview, int previewSteps,
      double touchGroundHeightMeters) {
    public Projectile {
      if (previewSteps < 1) {
        throw new IllegalArgumentException("previewSteps must be >= 1");
      }
      if (touchGroundHeightMeters < 0.0) {
        throw new IllegalArgumentException("touchGroundHeightMeters must be >= 0");
      }
    }
  }

  private final Timing timing;
  private final Collision collision;
  private final Boundaries boundaries;
  private final Aerodynamics aerodynamics;
  private final Projectile projectile;
  private final Vec3 gravityMps2;

  /**
   * Creates simulation options with explicit feature groups.
   */
  public SimulationOptions(Timing timing, Collision collision, Boundaries boundaries,
      Aerodynamics aerodynamics, Projectile projectile, Vec3 gravityMps2) {
    this.timing = Objects.requireNonNull(timing);
    this.collision = Objects.requireNonNull(collision);
    this.boundaries = Objects.requireNonNull(boundaries);
    this.aerodynamics = Objects.requireNonNull(aerodynamics);
    this.projectile = Objects.requireNonNull(projectile);
    this.gravityMps2 = Objects.requireNonNull(gravityMps2);
  }

  /**
   * Returns a practical default configuration suitable for FRC field simulation.
   */
  public static SimulationOptions defaults() {
    return new SimulationOptions(
        new Timing(0.004, 5),
        new Collision(true, 0.35, true),
        new Boundaries(16.54, 8.02, true),
        new Aerodynamics(true, true, 0.08, 0.03, 1.5),
        new Projectile(true, 100, 0.05),
        new Vec3(0.0, 0.0, -9.81));
  }

  public Timing timing() {
    return timing;
  }

  public Collision collision() {
    return collision;
  }

  public Boundaries boundaries() {
    return boundaries;
  }

  public Aerodynamics aerodynamics() {
    return aerodynamics;
  }

  public Projectile projectile() {
    return projectile;
  }

  public Vec3 gravityMps2() {
    return gravityMps2;
  }
}