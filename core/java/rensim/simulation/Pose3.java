package rensim.simulation;

import rensim.Vec3;

/**
 * Lightweight 3D pose for simulation telemetry.
 *
 * @param positionMeters position in meters
 * @param yawRad yaw angle in radians
 */
public record Pose3(Vec3 positionMeters, double yawRad) {
  /**
   * Returns a 2D projection pose.
   *
   * @return 2D projection of this pose
   */
  public Pose2 toPose2() {
    return new Pose2(positionMeters.x(), positionMeters.y(), yawRad);
  }
}