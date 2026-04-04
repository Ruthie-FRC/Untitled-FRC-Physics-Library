package rensim.simulation.projectile;

import java.util.Optional;
import rensim.Vec3;

/**
 * Ballistic targeting helper for computing launch velocity to hit a target position.
 */
public final class ProjectileTargetingSolver {
  private static final double EPS = 1.0e-9;

  private ProjectileTargetingSolver() {}

  /**
   * Computes a launch plan using fixed launch speed and gravity.
   */
  public static Optional<ProjectileLaunchPlan> solveBySpeed(Vec3 from, Vec3 to, double launchSpeedMps,
      double gravityMps2, boolean highArc) {
    if (!(launchSpeedMps > 0.0)) {
      throw new IllegalArgumentException("launchSpeedMps must be > 0");
    }
    if (gravityMps2 >= 0.0) {
      throw new IllegalArgumentException("gravityMps2 must be negative");
    }

    Vec3 delta = to.subtract(from);
    double dx = Math.hypot(delta.x(), delta.y());
    if (dx < EPS) {
      return Optional.empty();
    }

    double g = -gravityMps2;
    double v2 = launchSpeedMps * launchSpeedMps;
    double inside = v2 * v2 - g * (g * dx * dx + 2.0 * delta.z() * v2);
    if (inside < -EPS) {
      return Optional.empty();
    }
    inside = Math.max(inside, 0.0);

    double sqrt = Math.sqrt(inside);
    double tanTheta = (v2 + (highArc ? sqrt : -sqrt)) / (g * dx);
    double theta = Math.atan(tanTheta);

    double cos = Math.cos(theta);
    if (Math.abs(cos) < EPS) {
      return Optional.empty();
    }

    double planarScale = launchSpeedMps * cos;
    double ux = delta.x() / dx;
    double uy = delta.y() / dx;
    Vec3 velocity = new Vec3(ux * planarScale, uy * planarScale, launchSpeedMps * Math.sin(theta));
    double time = dx / planarScale;
    return Optional.of(new ProjectileLaunchPlan(velocity, time));
  }

  /**
   * Computes a launch plan with fixed angle and solved speed.
   */
  public static Optional<ProjectileLaunchPlan> solveByAngle(Vec3 from, Vec3 to, double launchAngleRad,
      double gravityMps2) {
    if (gravityMps2 >= 0.0) {
      throw new IllegalArgumentException("gravityMps2 must be negative");
    }
    Vec3 delta = to.subtract(from);
    double dx = Math.hypot(delta.x(), delta.y());
    if (dx < EPS) {
      return Optional.empty();
    }

    double tan = Math.tan(launchAngleRad);
    double cos = Math.cos(launchAngleRad);
    if (Math.abs(cos) < EPS) {
      return Optional.empty();
    }

    double g = -gravityMps2;
    double denom = 2.0 * cos * cos * (dx * tan - delta.z());
    if (denom <= EPS) {
      return Optional.empty();
    }

    double speed2 = (g * dx * dx) / denom;
    if (speed2 <= EPS) {
      return Optional.empty();
    }

    double speed = Math.sqrt(speed2);
    double ux = delta.x() / dx;
    double uy = delta.y() / dx;
    double planarScale = speed * cos;
    Vec3 velocity = new Vec3(ux * planarScale, uy * planarScale, speed * Math.sin(launchAngleRad));
    return Optional.of(new ProjectileLaunchPlan(velocity, dx / planarScale));
  }

  /**
   * Returns minimum launch speed required to hit target for the given gravity.
   */
  public static Optional<Double> minimumLaunchSpeed(Vec3 from, Vec3 to, double gravityMps2) {
    if (gravityMps2 >= 0.0) {
      throw new IllegalArgumentException("gravityMps2 must be negative");
    }
    Vec3 delta = to.subtract(from);
    double dx = Math.hypot(delta.x(), delta.y());
    if (dx < EPS) {
      return Optional.empty();
    }
    double g = -gravityMps2;
    double dz = delta.z();
    double root = Math.sqrt(dx * dx + dz * dz);
    double v2 = g * (dz + root);
    if (v2 <= EPS) {
      return Optional.empty();
    }
    return Optional.of(Math.sqrt(v2));
  }
}
