package rensim;

/**
 * Handle for a body managed by {@link PhysicsWorld}.
 */
public final class PhysicsBody {
  private final PhysicsWorld world;
  private final int bodyIndex;

  PhysicsBody(PhysicsWorld world, int bodyIndex) {
    this.world = world;
    this.bodyIndex = bodyIndex;
  }

  /**
   * Gets the native body index for this body.
   *
   * @return the native body index
   */
  public int bodyIndex() {
    return bodyIndex;
  }

  /**
   * Gets the body mass in kilograms.
   *
   * @return body mass in kilograms
   */
  public double massKg() {
    return world.getBodyMassKg(bodyIndex);
  }

  /**
   * Sets the body's world-space position in meters.
   *
   * @param positionMeters the new position in meters
   */
  public void setPosition(Vec3 positionMeters) {
    world.setBodyPosition(bodyIndex, positionMeters.x(), positionMeters.y(), positionMeters.z());
  }

  /**
   * Sets the body's linear velocity in meters per second.
   *
   * @param velocityMps the new linear velocity in meters per second
   */
  public void setLinearVelocity(Vec3 velocityMps) {
    world.setBodyLinearVelocity(bodyIndex, velocityMps.x(), velocityMps.y(), velocityMps.z());
  }

  /**
   * Enables or disables gravity for this body.
   *
   * @param enabled true to enable gravity
   */
  public void setGravityEnabled(boolean enabled) {
    world.setBodyGravityEnabled(bodyIndex, enabled);
  }

  /**
   * Assigns a sphere collider used by the Java-side starter collision solver.
   *
   * @param radiusMeters collider radius in meters
   * @param restitution coefficient of restitution in [0, 1]
   */
  public void setSphereCollider(double radiusMeters, double restitution) {
    world.setBodySphereCollider(bodyIndex, new SphereCollider(radiusMeters, restitution));
  }

  /**
   * Removes this body's sphere collider from the Java-side collision pass.
   */
  public void clearSphereCollider() {
    world.clearBodySphereCollider(bodyIndex);
  }

  /**
   * Gets the current world-space position in meters.
   *
   * @return the body position
   */
  public Vec3 position() {
    return world.getBodyPosition(bodyIndex);
  }

  /**
   * Gets the current world-space linear velocity in meters per second.
   *
   * @return the body linear velocity
   */
  public Vec3 linearVelocity() {
    return world.getBodyLinearVelocity(bodyIndex);
  }

  /**
   * Captures an immutable state snapshot for rendering/telemetry.
   *
   * @return body state snapshot
   */
  public BodyStateView state() {
    return world.getBodyState(bodyIndex);
  }
}
