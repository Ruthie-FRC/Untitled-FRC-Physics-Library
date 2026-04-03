package com.vendor.physics;

/**
 * Handle for a body managed by the native physics world.
 */
public final class PhysicsBody {
  private final frcsim_physics.PhysicsBody delegate;

  PhysicsBody(frcsim_physics.PhysicsBody delegate) {
    this.delegate = delegate;
  }

  /**
   * Gets the native body index for this body.
   *
   * @return the native body index
   */
  public int bodyIndex() {
    return delegate.bodyIndex();
  }

  /**
   * Sets the body's world-space position in meters.
   *
   * @param positionMeters the new position in meters
   */
  public void setPosition(Vec3 positionMeters) {
    delegate.setPosition(new frcsim_physics.Vec3(positionMeters.x(), positionMeters.y(), positionMeters.z()));
  }

  /**
   * Sets the body's linear velocity in meters per second.
   *
   * @param velocityMps the new linear velocity in meters per second
   */
  public void setLinearVelocity(Vec3 velocityMps) {
    delegate.setLinearVelocity(new frcsim_physics.Vec3(velocityMps.x(), velocityMps.y(), velocityMps.z()));
  }

  /**
   * Enables or disables gravity for this body.
   *
   * @param enabled true to enable gravity, false to disable it
   */
  public void setGravityEnabled(boolean enabled) {
    delegate.setGravityEnabled(enabled);
  }

  /**
   * Returns the body's current world-space position in meters.
   *
   * @return the current position in meters
   */
  public Vec3 position() {
    frcsim_physics.Vec3 position = delegate.position();
    return new Vec3(position.x(), position.y(), position.z());
  }

  /**
   * Returns the body's current linear velocity in meters per second.
   *
   * @return the current linear velocity in meters per second
   */
  public Vec3 linearVelocity() {
    frcsim_physics.Vec3 velocity = delegate.linearVelocity();
    return new Vec3(velocity.x(), velocity.y(), velocity.z());
  }
}
