// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

/**
 * Handle for a rectangular-prism hitbox managed by {@link PhysicsWorld}.
 *
 * <p>The prism is centered on its body position and uses the local x, y, and z axes for its
 * length, width, and height respectively.
 */
public final class RectangularPrism {
  private final PhysicsWorld world;
  private final int prismIndex;
  private final double lengthMeters;
  private final double widthMeters;
  private final double heightMeters;

  RectangularPrism(PhysicsWorld world, int prismIndex, double lengthMeters, double widthMeters,
      double heightMeters) {
    this.world = world;
    this.prismIndex = prismIndex;
    this.lengthMeters = lengthMeters;
    this.widthMeters = widthMeters;
    this.heightMeters = heightMeters;
  }

  /**
   * Gets the native body index backing this prism.
   *
   * @return the native body index
   */
  public int prismIndex() {
    return prismIndex;
  }

  /**
   * Gets the prism length in meters.
   *
   * @return prism length in meters
   */
  public double lengthMeters() {
    return lengthMeters;
  }

  /**
   * Gets the prism width in meters.
   *
   * @return prism width in meters
   */
  public double widthMeters() {
    return widthMeters;
  }

  /**
   * Gets the prism height in meters.
   *
   * @return prism height in meters
   */
  public double heightMeters() {
    return heightMeters;
  }

  /**
   * Sets the prism's world-space position in meters.
   *
   * @param pose the new position (only translation component is used)
   */
  public void setPosition(Pose3d pose) {
    Translation3d translation = pose.getTranslation();
    world.setRectangularPrismPosition(prismIndex, translation.getX(), translation.getY(), translation.getZ());
  }

  /**
   * Sets the prism's world-space position in meters.
   *
   * @param positionMeters the new position in meters
   */
  public void setPosition(Translation3d positionMeters) {
    world.setRectangularPrismPosition(prismIndex, positionMeters.getX(), positionMeters.getY(), positionMeters.getZ());
  }

  /**
   * Sets the prism's world-space position in meters.
   *
   * @param positionMeters the new position in meters
   * @deprecated use setPosition(Pose3d), setPosition(Translation3d), or setPosition(Distance, Distance, Distance)
   */
  @Deprecated(forRemoval = false)
  public void setPosition(Vec3 positionMeters) {
    world.setRectangularPrismPosition(prismIndex, positionMeters.x(), positionMeters.y(), positionMeters.z());
  }

  /**
   * Sets the prism's world-space position.
   *
   * @param x x position
   * @param y y position
   * @param z z position
   */
  public void setPosition(Distance x, Distance y, Distance z) {
    world.setRectangularPrismPosition(prismIndex, x.in(Meters), y.in(Meters), z.in(Meters));
  }

  /**
   * Sets the prism's world-space linear velocity in meters per second.
   *
   * @param velocityMps the new linear velocity in meters per second
   */
  public void setLinearVelocity(LinearVelocity3d velocityMps) {
    world.setRectangularPrismLinearVelocity(prismIndex, velocityMps.x(), velocityMps.y(), velocityMps.z());
  }

  /**
   * Sets the prism's world-space linear velocity in meters per second.
   *
   * @param velocityMps the new linear velocity in meters per second
   * @deprecated use setLinearVelocity(LinearVelocity3d)
   */
  @Deprecated(forRemoval = false)
  public void setLinearVelocity(Vec3 velocityMps) {
    world.setRectangularPrismLinearVelocity(prismIndex, velocityMps.x(), velocityMps.y(), velocityMps.z());
  }

  /**
   * Sets the prism's world-space linear velocity in meters per second.
   *
   * @param vxMetersPerSecond x velocity in meters per second
   * @param vyMetersPerSecond y velocity in meters per second
   * @param vzMetersPerSecond z velocity in meters per second
   */
  public void setLinearVelocity(
      double vxMetersPerSecond, double vyMetersPerSecond, double vzMetersPerSecond) {
    world.setRectangularPrismLinearVelocity(prismIndex, vxMetersPerSecond, vyMetersPerSecond, vzMetersPerSecond);
  }

  /**
   * Gets the current world-space position in meters.
   *
   * @return the prism position
   */
  public Pose3d position() {
    return world.getRectangularPrismPosition(prismIndex);
  }

  /**
   * Gets the current world-space linear velocity in meters per second.
   *
   * @return the prism linear velocity
   */
  public LinearVelocity3d linearVelocity() {
    return world.getRectangularPrismLinearVelocity(prismIndex);
  }
}