package rensim;

import rensim.jni.VendorJNI;

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
		VendorJNI.forceLoad();
		this.worldHandle = VendorJNI.createWorld(fixedDtSeconds, enableGravity);
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
		int index = VendorJNI.createBody(worldHandle, massKg);
		if (index < 0) {
			throw new IllegalStateException("Failed to create body");
		}
		return new PhysicsBody(this, index);
	}

	/**
	 * Sets the world's gravity vector in meters per second squared.
	 *
	 * @param gravityMps2 the gravity vector in meters per second squared
	 */
	public void setGravity(Vec3 gravityMps2) {
		ensureOpen();
		int rc = VendorJNI.setWorldGravity(worldHandle, gravityMps2.x(), gravityMps2.y(), gravityMps2.z());
		if (rc != 0) {
			throw new IllegalStateException("Failed to set world gravity");
		}
	}

	/** Advances the simulation by one step. */
	public void step() {
		step(1);
	}

	/**
	 * Advances the simulation by the requested number of steps.
	 *
	 * @param steps the number of simulation steps to advance
	 */
	public void step(int steps) {
		ensureOpen();
		int rc = VendorJNI.stepWorld(worldHandle, steps);
		if (rc != 0) {
			throw new IllegalStateException("Failed to step world");
		}
	}

	void setBodyPosition(int bodyIndex, Vec3 positionMeters) {
		ensureOpen();
		int rc = VendorJNI.setBodyPosition(worldHandle, bodyIndex, positionMeters.x(), positionMeters.y(), positionMeters.z());
		if (rc != 0) {
			throw new IllegalArgumentException("Invalid body index for setBodyPosition");
		}
	}

	void setBodyLinearVelocity(int bodyIndex, Vec3 velocityMps) {
		ensureOpen();
		int rc = VendorJNI.setBodyLinearVelocity(worldHandle, bodyIndex, velocityMps.x(), velocityMps.y(), velocityMps.z());
		if (rc != 0) {
			throw new IllegalArgumentException("Invalid body index for setBodyLinearVelocity");
		}
	}

	void setBodyGravityEnabled(int bodyIndex, boolean enabled) {
		ensureOpen();
		int rc = VendorJNI.setBodyGravityEnabled(worldHandle, bodyIndex, enabled);
		if (rc != 0) {
			throw new IllegalArgumentException("Invalid body index for setBodyGravityEnabled");
		}
	}

	Vec3 bodyPosition(int bodyIndex) {
		ensureOpen();
		double[] out = new double[3];
		int rc = VendorJNI.getBodyPosition(worldHandle, bodyIndex, out);
		if (rc != 0) {
			throw new IllegalArgumentException("Invalid body index for getBodyPosition");
		}
		return new Vec3(out[0], out[1], out[2]);
	}

	Vec3 bodyLinearVelocity(int bodyIndex) {
		ensureOpen();
		double[] out = new double[3];
		int rc = VendorJNI.getBodyLinearVelocity(worldHandle, bodyIndex, out);
		if (rc != 0) {
			throw new IllegalArgumentException("Invalid body index for getBodyLinearVelocity");
		}
		return new Vec3(out[0], out[1], out[2]);
	}

	/** Releases the native world handle. */
	@Override
	public void close() {
		if (worldHandle != 0) {
			VendorJNI.destroyWorld(worldHandle);
			worldHandle = 0;
		}
	}

	private void ensureOpen() {
		if (worldHandle == 0) {
			throw new IllegalStateException("PhysicsWorld is closed");
		}
	}
}