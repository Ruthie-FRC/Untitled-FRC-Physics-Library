package rensim.utils.mathutils;

import rensim.Vec3;
import rensim.simulation.Pose2;
import rensim.simulation.Pose3;

public final class GeometryConversions {
  private GeometryConversions() {}

  /**
   * Creates a Pose2 from a 3D pose using x/y and yaw.
   */
  public static Pose2 toPose2(Pose3 pose3) {
    return new Pose2(pose3.positionMeters().x(), pose3.positionMeters().y(), pose3.yawRad());
  }

  /**
   * Creates a 3D pose from planar values with zero roll/pitch.
   */
  public static Pose3 toPose3(Pose2 pose2, double zMeters) {
    return new Pose3(new Vec3(pose2.xMeters(), pose2.yMeters(), zMeters), pose2.yawRad());
  }

  public static Vec3 planarToVec3(Pose2 pose2, double zMeters) {
    return new Vec3(pose2.xMeters(), pose2.yMeters(), zMeters);
  }

  public static Pose2 vec3ToPlanarPose(Vec3 vec, double yawRad) {
    return new Pose2(vec.x(), vec.y(), yawRad);
  }
}