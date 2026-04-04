package rensim.simulation.projectile;

import java.util.Optional;
import rensim.Vec3;

/**
 * Ballistic targeting helper for computing launch velocity to hit a target position.
 */
public final class ProjectileTargetingSolver {
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
    if (dx < 1.0e-9) {
      return Optional.empty();
    }

    double g = -gravityMps2;
    double v2 = launchSpeedMps * launchSpeedMps;
    double inside = v2 * v2 - g * (g * dx * dx + 2.0 * delta.z() * v2);
    if (inside < 0.0) {
      return Optional.empty();
    }

    double sqrt = Math.sqrt(inside);
    double tanTheta = (v2 + (highArc ? sqrt : -sqrt)) / (g * dx);
    double theta = Math.atan(tanTheta);

    double cos = Math.cos(theta);
    if (Math.abs(cos) < 1.0e-9) {
      return Optional.empty();
    }

    double planarScale = launchSpeedMps * cos;
    double ux = delta.x() / dx;
    double uy = delta.y() / dx;
    Vec3 velocity = new Vec3(ux * planarScale, uy * planarScale, launchSpeedMps * Math.sin(theta));
    double time = dx / planarScale;
    return Optional.of(new ProjectileLaunchPlan(velocity, time));
  }
}
