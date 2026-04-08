// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package rensim;

import rensim.jni.RenSimJNI;

/**
 * Thin Java wrapper around the native physics world implementation.
 */
public final class PhysicsWorld implements AutoCloseable {
	private long worldHandle;

	/**
	 * Creates a native physics world.
	 *
	 * @param fixedDtSeconds fixed simulation timestep in seconds
	 * @param enableGravity whether gravity is enabled for newly created bodies
	 */
	public PhysicsWorld(double fixedDtSeconds, boolean enableGravity) {
		RenSimJNI.forceLoad();
		this.worldHandle = RenSimJNI.createWorld(fixedDtSeconds, enableGravity);
		if (worldHandle == 0) {
			throw new IllegalStateException("Failed to create native PhysicsWorld");
		}
	}

	/**
	 * Creates a new body with the provided mass in kilograms.
	 *
	 * @param massKg the body mass in kilograms
	 * @return the created body handle
	 */
	public PhysicsBody createBody(double massKg) {
		ensureOpen();
		int index = RenSimJNI.createBody(worldHandle, massKg);
		if (index < 0) {
			throw new IllegalStateException("Failed to create body");
		}
		return new PhysicsBody(this, index);
	}

	/**
	 * Sets the body's world-space position in meters.
	 *
	 * @param bodyIndex native body index
	 * @param xMeters x position in meters
	 * @param yMeters y position in meters
	 * @param zMeters z position in meters
	 */
	void setBodyPosition(int bodyIndex, double xMeters, double yMeters, double zMeters) {
		ensureOpen();
		int rc = RenSimJNI.setBodyPosition(worldHandle, bodyIndex, xMeters, yMeters, zMeters);
		if (rc != 0) {
			throw new IllegalStateException("Failed to set body position: rc=" + rc);
		}
	}

	/**
	 * Sets the body's linear velocity in meters per second.
	 *
	 * @param bodyIndex native body index
	 * @param vxMetersPerSecond x velocity in meters per second
	 * @param vyMetersPerSecond y velocity in meters per second
	 * @param vzMetersPerSecond z velocity in meters per second
	 */
	void setBodyLinearVelocity(int bodyIndex, double vxMetersPerSecond, double vyMetersPerSecond,
			double vzMetersPerSecond) {
		ensureOpen();
		int rc = RenSimJNI.setBodyLinearVelocity(worldHandle, bodyIndex, vxMetersPerSecond,
				vyMetersPerSecond, vzMetersPerSecond);
		if (rc != 0) {
			throw new IllegalStateException("Failed to set body linear velocity: rc=" + rc);
		}
	}

	/**
	 * Enables or disables gravity for the given body.
	 *
	 * @param bodyIndex native body index
	 * @param enabled true to enable gravity
	 */
	void setBodyGravityEnabled(int bodyIndex, boolean enabled) {
		ensureOpen();
		int rc = RenSimJNI.setBodyGravityEnabled(worldHandle, bodyIndex, enabled);
		if (rc != 0) {
			throw new IllegalStateException("Failed to set body gravity enabled: rc=" + rc);
		}
	}

	/**
	 * Gets the world position for the given body.
	 *
	 * @param bodyIndex native body index
	 * @return body position
	 */
	public Vec3 getBodyPosition(int bodyIndex) {
		ensureOpen();
		double[] values = new double[3];
		int rc = RenSimJNI.getBodyPosition(worldHandle, bodyIndex, values);
		if (rc != 0) {
			throw new IllegalStateException("Failed to get body position: rc=" + rc);
		}
		return new Vec3(values[0], values[1], values[2]);
	}

	/**
	 * Gets the world linear velocity for the given body.
	 *
	 * @param bodyIndex native body index
	 * @return body linear velocity
	 */
	public Vec3 getBodyLinearVelocity(int bodyIndex) {
		ensureOpen();
		double[] values = new double[3];
		int rc = RenSimJNI.getBodyLinearVelocity(worldHandle, bodyIndex, values);
		if (rc != 0) {
			throw new IllegalStateException("Failed to get body linear velocity: rc=" + rc);
		}
		return new Vec3(values[0], values[1], values[2]);
	}

	/**
	 * Advances the simulation by one step.
	 */
	public void step() {
		step(1);
	}

	/**
	 * Advances the simulation by the requested number of steps.
	 *
	 * @param steps number of steps to advance
	 */
	public void step(int steps) {
		ensureOpen();
		int rc = RenSimJNI.stepWorld(worldHandle, steps);
		if (rc != 0) {
			throw new IllegalStateException("Failed to step world: rc=" + rc);
		}
	}

	/**
	 * Applies a gravity vector to the world.
	 *
	 * @param gravity gravity vector in meters per second squared
	 */
	public void setGravity(Vec3 gravity) {
		ensureOpen();
		int rc = RenSimJNI.setWorldGravity(worldHandle, gravity.x(), gravity.y(), gravity.z());
		if (rc != 0) {
			throw new IllegalStateException("Failed to set gravity: rc=" + rc);
		}
	}

	@Override
	public void close() {
		if (worldHandle != 0) {
			RenSimJNI.destroyWorld(worldHandle);
			worldHandle = 0;
		}
	}

	private void ensureOpen() {
		if (worldHandle == 0) {
			throw new IllegalStateException("PhysicsWorld is closed");
		}
	}
}
