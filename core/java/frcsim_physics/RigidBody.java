package frcsim_physics;

import java.util.Objects;
import rensim.PhysicsBody;
import rensim.Vec3;

/**
 * High-level rigid-body wrapper for simulation gameplay logic and rendering.
 *
 * <p>This class layers orientation/angular state and force integration over the low-level
 * {@link PhysicsBody} translational state.
 */
public final class RigidBody {
	/**
	 * Immutable rendering pose.
	 *
	 * @param positionMeters world-space position
	 * @param orientation world-space orientation
	 */
	public record RenderPose(Vec3 positionMeters, Quaternion orientation) {}

	/**
	 * Quaternion implementation aligned with C++ frcsim::Quaternion behavior.
	 *
	 * @param w scalar component
	 * @param x x component
	 * @param y y component
	 * @param z z component
	 */
	public record Quaternion(double w, double x, double y, double z) {
		/** Identity rotation quaternion. */
		public static final Quaternion IDENTITY = new Quaternion(1.0, 0.0, 0.0, 0.0);

		/** Squared quaternion norm. */
		public double normSquared() {
			return (w * w) + (x * x) + (y * y) + (z * z);
		}

		/** Quaternion norm magnitude. */
		public double norm() {
			return Math.sqrt(normSquared());
		}

		/** Returns true for finite identity quaternion within epsilon. */
		public boolean isIdentity(double epsilon) {
			return Math.abs(w - 1.0) < epsilon && Math.abs(x) < epsilon && Math.abs(y) < epsilon
					&& Math.abs(z) < epsilon;
		}

		/** Returns true when any component is NaN. */
		public boolean hasNaN() {
			return Double.isNaN(w) || Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(z);
		}

		/**
		 * Returns a normalized quaternion.
		 *
		 * @return normalized quaternion
		 */
		public Quaternion normalized() {
			double n = norm();
			if (n < 1.0e-12) {
				return IDENTITY;
			}
			return new Quaternion(w / n, x / n, y / n, z / n);
		}

		/** Normalizes only when needed by epsilon threshold. */
		public Quaternion normalizeIfNeeded(double epsilon) {
			double n2 = normSquared();
			if (Math.abs(n2 - 1.0) <= epsilon) {
				return this;
			}
			return normalized();
		}

		/**
		 * Multiplies this quaternion by another quaternion.
		 *
		 * @param rhs right-hand quaternion
		 * @return quaternion product
		 */
		public Quaternion multiply(Quaternion rhs) {
			return new Quaternion(
					(w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z),
					(w * rhs.x) + (x * rhs.w) + (y * rhs.z) - (z * rhs.y),
					(w * rhs.y) - (x * rhs.z) + (y * rhs.w) + (z * rhs.x),
					(w * rhs.z) + (x * rhs.y) - (y * rhs.x) + (z * rhs.w));
		}

		/** Returns quaternion sum. */
		public Quaternion add(Quaternion rhs) {
			return new Quaternion(w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z);
		}

		/** Returns quaternion scaled by scalar. */
		public Quaternion scale(double scalar) {
			return new Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
		}

		/** Returns conjugate quaternion. */
		public Quaternion conjugate() {
			return new Quaternion(w, -x, -y, -z);
		}

		/** Returns quaternion inverse. */
		public Quaternion inverse() {
			double n2 = normSquared();
			if (n2 < 1.0e-12) {
				return IDENTITY;
			}
			Quaternion c = conjugate();
			return new Quaternion(c.w / n2, c.x / n2, c.y / n2, c.z / n2);
		}

		/** Rotates vector using q * v * q_conjugate. */
		public Vec3 rotate(Vec3 v) {
			Quaternion qv = new Quaternion(0.0, v.x(), v.y(), v.z());
			Quaternion res = multiply(qv).multiply(conjugate());
			return new Vec3(res.x, res.y, res.z);
		}

		/** Builds quaternion from axis-angle. */
		public static Quaternion fromAxisAngle(Vec3 axis, double angleRad) {
			Vec3 nAxis = axis.normalized();
			double half = 0.5 * angleRad;
			double s = Math.sin(half);
			return new Quaternion(Math.cos(half), nAxis.x() * s, nAxis.y() * s, nAxis.z() * s);
		}

		/** Builds quaternion from angular velocity over dt. */
		public static Quaternion fromAngularVelocity(Vec3 omegaRadPerSec, double dtSeconds) {
			double angle = omegaRadPerSec.norm() * dtSeconds;
			Vec3 axis = angle > 1.0e-12 ? omegaRadPerSec.normalized() : Vec3.unitX();
			return fromAxisAngle(axis, angle);
		}

		/**
		 * Integrates orientation from angular velocity.
		 *
		 * @param angularVelocityRadPerSec world-space angular velocity
		 * @param dtSeconds timestep in seconds
		 * @return updated orientation
		 */
		public Quaternion integrate(Vec3 angularVelocityRadPerSec, double dtSeconds) {
			Quaternion omega = new Quaternion(
					0.0,
					angularVelocityRadPerSec.x(),
					angularVelocityRadPerSec.y(),
					angularVelocityRadPerSec.z());
			Quaternion dq = omega.multiply(this).scale(0.5);
			Quaternion integrated = add(dq.scale(dtSeconds));
			return integrated.normalizeIfNeeded(1.0e-12);
		}
	}

	private final PhysicsBody body;
	private Vec3 angularVelocityRadPerSec = Vec3.ZERO;
	private Vec3 accumulatedForceN = Vec3.ZERO;
	private Vec3 accumulatedTorqueNm = Vec3.ZERO;
	private Quaternion orientation = Quaternion.IDENTITY;
	private double linearDamping = 0.05;
	private double angularDamping = 0.05;

	/**
	 * Creates a rigid-body wrapper over a low-level body handle.
	 *
	 * @param body low-level body handle
	 */
	public RigidBody(PhysicsBody body) {
		this.body = Objects.requireNonNull(body);
	}

	/**
	 * Returns the wrapped low-level body.
	 *
	 * @return wrapped body handle
	 */
	public PhysicsBody body() {
		return body;
	}

	/**
	 * Returns the current world position in meters.
	 *
	 * @return body position in meters
	 */
	public Vec3 positionMeters() {
		return body.position();
	}

	/**
	 * Sets world-space body position in meters.
	 *
	 * @param positionMeters new world position
	 */
	public void setPositionMeters(Vec3 positionMeters) {
		body.setPosition(positionMeters);
	}

	/**
	 * Returns linear velocity in meters per second.
	 *
	 * @return linear velocity
	 */
	public Vec3 linearVelocityMps() {
		return body.linearVelocity();
	}

	/**
	 * Sets linear velocity in meters per second.
	 *
	 * @param velocityMps world linear velocity
	 */
	public void setLinearVelocityMps(Vec3 velocityMps) {
		body.setLinearVelocity(velocityMps);
	}

	/**
	 * Returns angular velocity in radians per second.
	 *
	 * @return angular velocity
	 */
	public Vec3 angularVelocityRadPerSec() {
		return angularVelocityRadPerSec;
	}

	/**
	 * Sets angular velocity in radians per second.
	 *
	 * @param angularVelocityRadPerSec angular velocity
	 */
	public void setAngularVelocityRadPerSec(Vec3 angularVelocityRadPerSec) {
		this.angularVelocityRadPerSec = Objects.requireNonNull(angularVelocityRadPerSec);
	}

	/**
	 * Returns orientation quaternion for graphics.
	 *
	 * @return world orientation
	 */
	public Quaternion orientation() {
		return orientation;
	}

	/**
	 * Explicitly sets orientation quaternion.
	 *
	 * @param orientation new orientation
	 */
	public void setOrientation(Quaternion orientation) {
		this.orientation = Objects.requireNonNull(orientation).normalized();
	}

	/**
	 * Sets linear damping factor in 1/s.
	 *
	 * @param damping linear damping coefficient
	 */
	public void setLinearDamping(double damping) {
		if (damping < 0.0) {
			throw new IllegalArgumentException("damping must be >= 0");
		}
		this.linearDamping = damping;
	}

	/**
	 * Sets angular damping factor in 1/s.
	 *
	 * @param damping angular damping coefficient
	 */
	public void setAngularDamping(double damping) {
		if (damping < 0.0) {
			throw new IllegalArgumentException("damping must be >= 0");
		}
		this.angularDamping = damping;
	}

	/**
	 * Adds a world-space force to be integrated on next advance.
	 *
	 * @param forceNewton force vector in Newtons
	 */
	public void addForce(Vec3 forceNewton) {
		accumulatedForceN = accumulatedForceN.add(Objects.requireNonNull(forceNewton));
	}

	/**
	 * Adds a world-space torque to be integrated on next advance.
	 *
	 * @param torqueNewtonMeters torque in N*m
	 */
	public void addTorque(Vec3 torqueNewtonMeters) {
		accumulatedTorqueNm = accumulatedTorqueNm.add(Objects.requireNonNull(torqueNewtonMeters));
	}

	/**
	 * Integrates accumulated forces/torques and updates orientation.
	 *
	 * @param dtSeconds timestep in seconds
	 */
	public void advance(double dtSeconds) {
		if (!(dtSeconds > 0.0)) {
			throw new IllegalArgumentException("dtSeconds must be > 0");
		}

		double massKg = body.massKg();
		Vec3 linearAcceleration = accumulatedForceN.scale(1.0 / massKg);
		Vec3 linearVelocity = body.linearVelocity().add(linearAcceleration.scale(dtSeconds));
		linearVelocity = linearVelocity.scale(Math.exp(-linearDamping * dtSeconds));
		body.setLinearVelocity(linearVelocity);

		Vec3 angularAcceleration = accumulatedTorqueNm.scale(1.0 / massKg);
		angularVelocityRadPerSec = angularVelocityRadPerSec.add(angularAcceleration.scale(dtSeconds));
		angularVelocityRadPerSec = angularVelocityRadPerSec.scale(Math.exp(-angularDamping * dtSeconds));
		orientation = orientation.integrate(angularVelocityRadPerSec, dtSeconds);

		accumulatedForceN = Vec3.ZERO;
		accumulatedTorqueNm = Vec3.ZERO;
	}

	/**
	 * Captures a graphics-facing render pose.
	 *
	 * @return immutable render pose
	 */
	public RenderPose renderPose() {
		return new RenderPose(body.position(), orientation);
	}

	/**
	 * Applies a simple pairwise sphere collision response between two rigid bodies.
	 *
	 * @param a first body
	 * @param radiusA sphere radius of body A in meters
	 * @param b second body
	 * @param radiusB sphere radius of body B in meters
	 * @param restitution restitution coefficient in [0, 1]
	 * @return true when a collision was detected and resolved
	 */
	public static boolean resolveSphereCollision(
			RigidBody a,
			double radiusA,
			RigidBody b,
			double radiusB,
			double restitution) {
		if (radiusA <= 0.0 || radiusB <= 0.0) {
			throw new IllegalArgumentException("Sphere radii must be > 0");
		}
		if (restitution < 0.0 || restitution > 1.0) {
			throw new IllegalArgumentException("restitution must be in [0, 1]");
		}

		Vec3 pa = a.positionMeters();
		Vec3 pb = b.positionMeters();
		Vec3 delta = pb.subtract(pa);
		double distance = delta.norm();
		double minDistance = radiusA + radiusB;
		if (distance >= minDistance) {
			return false;
		}

		Vec3 normal = (distance < 1.0e-9) ? new Vec3(1.0, 0.0, 0.0) : delta.scale(1.0 / distance);
		double invMassA = 1.0 / a.body.massKg();
		double invMassB = 1.0 / b.body.massKg();
		double invMassSum = invMassA + invMassB;
		if (invMassSum <= 0.0) {
			return false;
		}

		Vec3 va = a.linearVelocityMps();
		Vec3 vb = b.linearVelocityMps();
		double normalVel = vb.subtract(va).dot(normal);
		if (normalVel < 0.0) {
			double impulseMag = -((1.0 + restitution) * normalVel) / invMassSum;
			Vec3 impulse = normal.scale(impulseMag);
			a.setLinearVelocityMps(va.subtract(impulse.scale(invMassA)));
			b.setLinearVelocityMps(vb.add(impulse.scale(invMassB)));
		}

		double penetration = minDistance - distance;
		if (penetration > 0.0) {
			Vec3 correction = normal.scale((penetration / invMassSum) * 0.8);
			a.setPositionMeters(pa.subtract(correction.scale(invMassA)));
			b.setPositionMeters(pb.add(correction.scale(invMassB)));
		}

		return true;
	}
}
