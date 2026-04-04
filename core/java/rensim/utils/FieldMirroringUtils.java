package rensim.utils;

import rensim.simulation.Pose2;
import rensim.Vec3;

/**
 * Utility helpers for mirroring field coordinates between blue and red alliances.
 */
public final class FieldMirroringUtils {
  public static final double FIELD_WIDTH_METERS = 16.54;

  private FieldMirroringUtils() {}

  /**
   * Mirrors a 2D pose across the field center line along X.
   */
  public static Pose2 flip(Pose2 pose) {
    return new Pose2(FIELD_WIDTH_METERS - pose.xMeters(), pose.yMeters(),
        normalizeAngle(Math.PI - pose.yawRad()));
  }

  /**
   * Mirrors a 3D translation (x mirrored, y/z retained).
   */
  public static Vec3 flip(Vec3 translation) {
    return new Vec3(FIELD_WIDTH_METERS - translation.x(), translation.y(), translation.z());
  }

  private static double normalizeAngle(double angle) {
    double a = angle;
    while (a > Math.PI) {
      a -= 2.0 * Math.PI;
    }
    while (a < -Math.PI) {
      a += 2.0 * Math.PI;
    }
    return a;
  }
}