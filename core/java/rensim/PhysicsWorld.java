package rensim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import rensim.jni.VendorJNI;

/**
 * Thin Java wrapper around the native physics world implementation.
 */
public final class PhysicsWorld implements AutoCloseable {
	private long worldHandle;
	private final double fixedDtSeconds;
	private final boolean defaultBodyGravityEnabled;
	private final List<Integer> bodyIndices = new ArrayList<>();
	private final Map<Integer, Double> bodyMassKgById = new LinkedHashMap<>();
	private final Map<Integer, Boolean> bodyGravityEnabledById = new LinkedHashMap<>();
	private final Map<Integer, SphereCollider> sphereCollidersById = new LinkedHashMap<>();
	private boolean simpleSphereCollisionsEnabled;
	private CollisionListener collisionListener = CollisionListener.noop();

	/**
	 * Creates a native physics world.
	 *
	 * @param fixedDtSeconds fixed simulation timestep in seconds
	 * @param enableGravity whether gravity is enabled for newly created bodies
	 */
	public PhysicsWorld(double fixedDtSeconds, boolean enableGravity) {
		VendorJNI.forceLoad();
		if (!(fixedDtSeconds > 0.0)) {
			throw new IllegalArgumentException("fixedDtSeconds must be > 0");
		}
		this.fixedDtSeconds = fixedDtSeconds;
		this.defaultBodyGravityEnabled = enableGravity;
		this.worldHandle = VendorJNI.createWorld(fixedDtSeconds, enableGravity);
		if (worldHandle == 0) {
			throw new IllegalStateException("Failed to create native PhysicsWorld");
		}
	}

	/**
	 * Gets this world's fixed timestep in seconds.
	 *
	 * @return fixed timestep in seconds
	 */
	public double fixedDtSeconds() {
		return fixedDtSeconds;
	}

	/**
	 * Creates a new body with the provided mass in kilograms.
	 *
	 * @param massKg the body mass in kilograms
	 * @return the created body handle
	 */
	public PhysicsBody createBody(double massKg) {
		ensureOpen();
		if (!(massKg > 0.0)) {
			throw new IllegalArgumentException("massKg must be > 0");
		}
		int index = VendorJNI.createBody(worldHandle, massKg);
		if (index < 0) {
			throw new IllegalStateException("Failed to create body");
		}
		bodyIndices.add(index);
		bodyMassKgById.put(index, massKg);
		bodyGravityEnabledById.put(index, defaultBodyGravityEnabled);
		return new PhysicsBody(this, index);
	}

	/**
	 * Returns the mass of a body in kilograms.
	 *
	 * @param bodyIndex native body index
	 * @return body mass in kilograms
	 */
	double getBodyMassKg(int bodyIndex) {
		ensureOpen();
		ensureBodyKnown(bodyIndex);
		return bodyMassKgById.get(bodyIndex);
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
		ensureBodyKnown(bodyIndex);
		int rc = VendorJNI.setBodyPosition(worldHandle, bodyIndex, xMeters, yMeters, zMeters);
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
		ensureBodyKnown(bodyIndex);
		int rc = VendorJNI.setBodyLinearVelocity(worldHandle, bodyIndex, vxMetersPerSecond,
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
		ensureBodyKnown(bodyIndex);
		int rc = VendorJNI.setBodyGravityEnabled(worldHandle, bodyIndex, enabled);
		if (rc != 0) {
			throw new IllegalStateException("Failed to set body gravity enabled: rc=" + rc);
		}
		bodyGravityEnabledById.put(bodyIndex, enabled);
	}

	/**
	 * Assigns a sphere collider to a body for the Java-side starter collision pass.
	 *
	 * @param bodyIndex native body index
	 * @param collider sphere collider settings
	 */
	void setBodySphereCollider(int bodyIndex, SphereCollider collider) {
		ensureOpen();
		ensureBodyKnown(bodyIndex);
		sphereCollidersById.put(bodyIndex, Objects.requireNonNull(collider));
	}

	/**
	 * Removes a sphere collider from a body.
	 *
	 * @param bodyIndex native body index
	 */
	void clearBodySphereCollider(int bodyIndex) {
		ensureOpen();
		ensureBodyKnown(bodyIndex);
		sphereCollidersById.remove(bodyIndex);
	}

	/**
	 * Enables or disables the Java-side starter sphere collision pass.
	 *
	 * @param enabled true to enable sphere collision handling
	 */
	public void setSimpleSphereCollisionsEnabled(boolean enabled) {
		ensureOpen();
		simpleSphereCollisionsEnabled = enabled;
	}

	/**
	 * Returns whether Java-side sphere collisions are enabled.
	 *
	 * @return true if enabled
	 */
	public boolean simpleSphereCollisionsEnabled() {
		return simpleSphereCollisionsEnabled;
	}

	/**
	 * Sets a listener for collision contacts emitted by Java-side collision handling.
	 *
	 * @param listener collision listener
	 */
	public void setCollisionListener(CollisionListener listener) {
		collisionListener = Objects.requireNonNull(listener);
	}

	/**
	 * Gets the world position for the given body.
	 *
	 * @param bodyIndex native body index
	 * @return body position
	 */
	public Vec3 getBodyPosition(int bodyIndex) {
		ensureOpen();
		ensureBodyKnown(bodyIndex);
		double[] values = new double[3];
		int rc = VendorJNI.getBodyPosition(worldHandle, bodyIndex, values);
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
		ensureBodyKnown(bodyIndex);
		double[] values = new double[3];
		int rc = VendorJNI.getBodyLinearVelocity(worldHandle, bodyIndex, values);
		if (rc != 0) {
			throw new IllegalStateException("Failed to get body linear velocity: rc=" + rc);
		}
		return new Vec3(values[0], values[1], values[2]);
	}

	/**
	 * Captures an immutable snapshot of a body's current state.
	 *
	 * @param bodyIndex native body index
	 * @return immutable body state
	 */
	public BodyStateView getBodyState(int bodyIndex) {
		ensureOpen();
		ensureBodyKnown(bodyIndex);
		return new BodyStateImpl(
				bodyIndex,
				bodyMassKgById.get(bodyIndex),
				getBodyPosition(bodyIndex),
				getBodyLinearVelocity(bodyIndex),
				bodyGravityEnabledById.getOrDefault(bodyIndex, defaultBodyGravityEnabled));
	}

	/**
	 * Captures immutable snapshots for all bodies, preserving creation order.
	 *
	 * @return immutable list of body states
	 */
	public List<BodyStateView> snapshotBodies() {
		ensureOpen();
		List<BodyStateView> snapshots = new ArrayList<>(bodyIndices.size());
		for (int bodyIndex : bodyIndices) {
			snapshots.add(getBodyState(bodyIndex));
		}
		return List.copyOf(snapshots);
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
		if (steps < 1) {
			throw new IllegalArgumentException("steps must be >= 1");
		}
		for (int i = 0; i < steps; i++) {
			int rc = VendorJNI.stepWorld(worldHandle, 1);
			if (rc != 0) {
				throw new IllegalStateException("Failed to step world: rc=" + rc);
			}
			if (simpleSphereCollisionsEnabled && sphereCollidersById.size() > 1) {
				resolveSimpleSphereCollisions();
			}
		}
	}

	/**
	 * Applies a gravity vector to the world.
	 *
	 * @param gravity gravity vector in meters per second squared
	 */
	public void setGravity(Vec3 gravity) {
		ensureOpen();
		int rc = VendorJNI.setWorldGravity(worldHandle, gravity.x(), gravity.y(), gravity.z());
		if (rc != 0) {
			throw new IllegalStateException("Failed to set gravity: rc=" + rc);
		}
	}

	@Override
	public void close() {
		if (worldHandle != 0) {
			VendorJNI.destroyWorld(worldHandle);
			worldHandle = 0;
			bodyIndices.clear();
			bodyMassKgById.clear();
			bodyGravityEnabledById.clear();
			sphereCollidersById.clear();
		}
	}

	private void ensureOpen() {
		if (worldHandle == 0) {
			throw new IllegalStateException("PhysicsWorld is closed");
		}
	}

	private void ensureBodyKnown(int bodyIndex) {
		if (!bodyMassKgById.containsKey(bodyIndex)) {
			throw new IllegalArgumentException("Unknown body index: " + bodyIndex);
		}
	}

	private void resolveSimpleSphereCollisions() {
		for (int i = 0; i < bodyIndices.size(); i++) {
			int idA = bodyIndices.get(i);
			SphereCollider sphereA = sphereCollidersById.get(idA);
			if (sphereA == null) {
				continue;
			}

			for (int j = i + 1; j < bodyIndices.size(); j++) {
				int idB = bodyIndices.get(j);
				SphereCollider sphereB = sphereCollidersById.get(idB);
				if (sphereB == null) {
					continue;
				}

				Vec3 posA = getBodyPosition(idA);
				Vec3 posB = getBodyPosition(idB);
				Vec3 delta = posB.subtract(posA);
				double distance = delta.norm();
				double combinedRadius = sphereA.radiusMeters() + sphereB.radiusMeters();
				if (distance >= combinedRadius) {
					continue;
				}

				Vec3 normal;
				if (distance < 1.0e-9) {
					normal = new Vec3(1.0, 0.0, 0.0);
					distance = 0.0;
				} else {
					normal = delta.scale(1.0 / distance);
				}

				double invMassA = 1.0 / bodyMassKgById.get(idA);
				double invMassB = 1.0 / bodyMassKgById.get(idB);
				double invMassSum = invMassA + invMassB;
				if (invMassSum <= 0.0) {
					continue;
				}

				Vec3 velA = getBodyLinearVelocity(idA);
				Vec3 velB = getBodyLinearVelocity(idB);
				Vec3 relativeVelocity = velB.subtract(velA);
				double normalVelocity = relativeVelocity.dot(normal);

				double restitution = Math.min(sphereA.restitution(), sphereB.restitution());
				double impulseMag = 0.0;
				if (normalVelocity < 0.0) {
					impulseMag = -(1.0 + restitution) * normalVelocity / invMassSum;
					Vec3 impulse = normal.scale(impulseMag);
					Vec3 nextVelA = velA.subtract(impulse.scale(invMassA));
					Vec3 nextVelB = velB.add(impulse.scale(invMassB));
					setBodyLinearVelocity(idA, nextVelA.x(), nextVelA.y(), nextVelA.z());
					setBodyLinearVelocity(idB, nextVelB.x(), nextVelB.y(), nextVelB.z());
				}

				double penetration = combinedRadius - distance;
				if (penetration > 0.0) {
					double correctionScale = (Math.max(penetration - 1.0e-4, 0.0) / invMassSum) * 0.8;
					Vec3 correction = normal.scale(correctionScale);
					Vec3 nextPosA = posA.subtract(correction.scale(invMassA));
					Vec3 nextPosB = posB.add(correction.scale(invMassB));
					setBodyPosition(idA, nextPosA.x(), nextPosA.y(), nextPosA.z());
					setBodyPosition(idB, nextPosB.x(), nextPosB.y(), nextPosB.z());
				}

				collisionListener.onContact(new CollisionContact(idA, idB, normal, penetration,
						normalVelocity, impulseMag));
			}
		}
	}
}
