package frcsim_physics;

import java.util.Objects;

import rensim.Vec3;

/**
 * Reduced-order rigid body model for Java-side simulation and planning code.
 *
 * <p>This class stores the same core state a physics engine would expose: mass,
 * position, linear velocity, angular velocity, and force/torque accumulators.
 * It is intentionally self-contained and does not require JNI.
 */
public final class RigidBody {
	private double massKg;
	private double momentOfInertiaKgM2;

	private Vec3 positionMeters = Vec3.ZERO;
	private Vec3 linearVelocityMps = Vec3.ZERO;
	private Vec3 angularVelocityRadps = Vec3.ZERO;

	private Vec3 accumulatedForceNewtons = Vec3.ZERO;
	private Vec3 accumulatedTorqueNewtonMeters = Vec3.ZERO;

	private boolean gravityEnabled = true;

	/**
	 * Creates a body with the provided mass.
	 *
	 * @param massKg body mass in kilograms
	 */
	public RigidBody(double massKg) {
		setMassKg(massKg);
		momentOfInertiaKgM2 = this.massKg;
	}

	/**
	 * Returns the body mass in kilograms.
	 *
	 * @return mass in kilograms
	 */
	public double massKg() {
		return massKg;
	}

	/**
	 * Sets the body mass in kilograms.
	 *
	 * @param massKg new mass in kilograms; non-positive values are clamped to 1.0
	 */
	public void setMassKg(double massKg) {
		this.massKg = massKg > 0.0 ? massKg : 1.0;
		if (momentOfInertiaKgM2 <= 0.0) {
			momentOfInertiaKgM2 = this.massKg;
		}
	}

	/**
	 * Returns the body's inverse mass.
	 *
	 * @return inverse mass in 1/kg
	 */
	public double inverseMass() {
		return 1.0 / massKg;
	}

	/**
	 * Returns the body moment of inertia used for the stored angular velocity.
	 *
	 * @return moment of inertia in kg*m^2
	 */
	public double momentOfInertiaKgM2() {
		return momentOfInertiaKgM2;
	}

	/**
	 * Sets the scalar moment of inertia used for angular integration.
	 *
	 * @param momentOfInertiaKgM2 moment of inertia in kg*m^2; non-positive values are clamped to 1.0
	 */
	public void setMomentOfInertiaKgM2(double momentOfInertiaKgM2) {
		this.momentOfInertiaKgM2 = momentOfInertiaKgM2 > 0.0 ? momentOfInertiaKgM2 : 1.0;
	}

	/**
	 * Returns the world-space position in meters.
	 *
	 * @return position in meters
	 */
	public Vec3 position() {
		return positionMeters;
	}

	/**
	 * Sets the world-space position in meters.
	 *
	 * @param positionMeters new position in meters
	 */
	public void setPosition(Vec3 positionMeters) {
		this.positionMeters = requireVec(positionMeters, "positionMeters");
	}

	/**
	 * Returns the linear velocity in meters per second.
	 *
	 * @return linear velocity in meters per second
	 */
	public Vec3 linearVelocity() {
		return linearVelocityMps;
	}

	/**
	 * Sets the linear velocity in meters per second.
	 *
	 * @param linearVelocityMps new linear velocity in meters per second
	 */
	public void setLinearVelocity(Vec3 linearVelocityMps) {
		this.linearVelocityMps = requireVec(linearVelocityMps, "linearVelocityMps");
	}

	/**
	 * Returns the angular velocity in radians per second.
	 *
	 * @return angular velocity in radians per second
	 */
	public Vec3 angularVelocity() {
		return angularVelocityRadps;
	}

	/**
	 * Sets the angular velocity in radians per second.
	 *
	 * @param angularVelocityRadps new angular velocity in radians per second
	 */
	public void setAngularVelocity(Vec3 angularVelocityRadps) {
		this.angularVelocityRadps = requireVec(angularVelocityRadps, "angularVelocityRadps");
	}

	/**
	 * Returns whether gravity contributes to this body during integration.
	 *
	 * @return true if gravity is enabled
	 */
	public boolean gravityEnabled() {
		return gravityEnabled;
	}

	/**
	 * Enables or disables gravity for this body.
	 *
	 * @param gravityEnabled true to enable gravity, false to disable it
	 */
	public void setGravityEnabled(boolean gravityEnabled) {
		this.gravityEnabled = gravityEnabled;
	}

	/**
	 * Returns the currently accumulated force in newtons.
	 *
	 * @return accumulated force in newtons
	 */
	public Vec3 accumulatedForce() {
		return accumulatedForceNewtons;
	}

	/**
	 * Returns the currently accumulated torque in newton-meters.
	 *
	 * @return accumulated torque in newton-meters
	 */
	public Vec3 accumulatedTorque() {
		return accumulatedTorqueNewtonMeters;
	}

	/**
	 * Applies a force at the body center of mass.
	 *
	 * @param forceNewtons force in newtons
	 */
	public void applyForce(Vec3 forceNewtons) {
		accumulatedForceNewtons = add(accumulatedForceNewtons, requireVec(forceNewtons, "forceNewtons"));
	}

	/**
	 * Applies a torque to the body.
	 *
	 * @param torqueNewtonMeters torque in newton-meters
	 */
	public void applyTorque(Vec3 torqueNewtonMeters) {
		accumulatedTorqueNewtonMeters = add(accumulatedTorqueNewtonMeters,
				requireVec(torqueNewtonMeters, "torqueNewtonMeters"));
	}

	/**
	 * Applies a force at a world-space point, generating both force and torque.
	 *
	 * @param forceNewtons force in newtons
	 * @param worldPointMeters application point in world coordinates
	 */
	public void applyForceAtPoint(Vec3 forceNewtons, Vec3 worldPointMeters) {
		final Vec3 force = requireVec(forceNewtons, "forceNewtons");
		final Vec3 worldPoint = requireVec(worldPointMeters, "worldPointMeters");

		applyForce(force);
		final Vec3 r = subtract(worldPoint, positionMeters);
		applyTorque(cross(r, force));
	}

	/**
	 * Applies a linear impulse to the body.
	 *
	 * @param impulseNewtonSeconds impulse in newton-seconds
	 */
	public void applyImpulse(Vec3 impulseNewtonSeconds) {
		final Vec3 impulse = requireVec(impulseNewtonSeconds, "impulseNewtonSeconds");
		linearVelocityMps = add(linearVelocityMps, scale(impulse, inverseMass()));
	}

	/**
	 * Applies an angular impulse to the body.
	 *
	 * @param impulseNewtonMeterSeconds impulse in newton-meter-seconds
	 */
	public void applyAngularImpulse(Vec3 impulseNewtonMeterSeconds) {
		final Vec3 impulse = requireVec(impulseNewtonMeterSeconds, "impulseNewtonMeterSeconds");
		angularVelocityRadps = add(angularVelocityRadps, scale(impulse, 1.0 / momentOfInertiaKgM2));
	}

	/**
	 * Clears force and torque accumulators.
	 */
	public void clearAccumulators() {
		accumulatedForceNewtons = Vec3.ZERO;
		accumulatedTorqueNewtonMeters = Vec3.ZERO;
	}

	/**
	 * Advances the body state using semi-implicit Euler integration.
	 *
	 * @param dtSeconds simulation timestep in seconds
	 * @param gravityMps2 gravity vector in meters per second squared
	 */
	public void integrate(double dtSeconds, Vec3 gravityMps2) {
		if (dtSeconds <= 0.0) {
			return;
		}

		final Vec3 gravity = requireVec(gravityMps2, "gravityMps2");

		Vec3 totalForce = accumulatedForceNewtons;
		if (gravityEnabled) {
			totalForce = add(totalForce, scale(gravity, massKg));
		}

		final Vec3 linearAcceleration = scale(totalForce, inverseMass());
		linearVelocityMps = add(linearVelocityMps, scale(linearAcceleration, dtSeconds));
		positionMeters = add(positionMeters, scale(linearVelocityMps, dtSeconds));

		final Vec3 angularAcceleration = scale(accumulatedTorqueNewtonMeters, 1.0 / momentOfInertiaKgM2);
		angularVelocityRadps = add(angularVelocityRadps, scale(angularAcceleration, dtSeconds));

		clearAccumulators();
	}

	private static Vec3 requireVec(Vec3 vector, String name) {
		return Objects.requireNonNull(vector, name + " cannot be null");
	}

	private static Vec3 add(Vec3 a, Vec3 b) {
		return new Vec3(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
	}

	private static Vec3 subtract(Vec3 a, Vec3 b) {
		return new Vec3(a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
	}

	private static Vec3 scale(Vec3 vector, double scalar) {
		return new Vec3(vector.x() * scalar, vector.y() * scalar, vector.z() * scalar);
	}

	private static Vec3 cross(Vec3 a, Vec3 b) {
		return new Vec3(
				a.y() * b.z() - a.z() * b.y(),
				a.z() * b.x() - a.x() * b.z(),
				a.x() * b.y() - a.y() * b.x());
	}
}
