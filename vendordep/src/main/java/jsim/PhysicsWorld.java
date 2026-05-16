// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import jsim.jni.JSimJNI;

/**
 * Thin Java wrapper around the native physics world implementation.
 */
public final class PhysicsWorld implements AutoCloseable {
	private long worldHandle;
	private final List<Ball> balls = new ArrayList<>();
	private final List<RectangularPrism> rectangularPrisms = new ArrayList<>();
	private final List<Runnable> stepListeners = new ArrayList<>();

	/**
	 * Creates a native physics world.
	 *
	 * @param fixedDtSeconds fixed simulation timestep in seconds
	 * @param enableGravity whether gravity is enabled for newly created bodies
	 */
	public PhysicsWorld(double fixedDtSeconds, boolean enableGravity) {
		JSimJNI.forceLoad();
		this.worldHandle = JSimJNI.createWorld(fixedDtSeconds, enableGravity);
		if (worldHandle == 0) {
			throw new JSimException("Failed to create native PhysicsWorld", 0,
				"Ensure the native JSim library is available and that permissions allow loading it.");
		}
	}

	/**
	 * Creates a new body with the provided mass in kilograms.
	 *
	 * @param massKg the body mass in kilograms
	 * @return the created body handle
	 */
	public PhysicsBody createBody(double massKg) {
		int index = JSimJNI.createBody(worldHandle, massKg);
		if (index < 0) {
			throw new JSimException("Failed to create body with mass=" + massKg + " kg", index,
				"Body mass must be a positive, finite value. Received: " + massKg);
		}
		return new PhysicsBody(this, index);
	}

	/**
	 * Creates a new ball in the world.
	 *
	 * @return the created ball handle
	 */
	public Ball createBall() {
		int index = JSimJNI.createBall(worldHandle);
		if (index < 0) {
			throw new JSimException("Failed to create ball in physics world", index,
				"The physics world may have reached its maximum capacity or internal resources are exhausted. Ensure the world is properly initialized.");
		}
		Ball ball = new Ball(this, index);
		balls.add(ball);
		return ball;
	}

	/**
	 * Creates a new rectangular-prism hitbox in the world.
	 *
	 * @param lengthMeters prism length in meters along the local x axis
	 * @param widthMeters prism width in meters along the local y axis
	 * @param heightMeters prism height in meters along the local z axis
	 * @return the created rectangular-prism handle
	 */
	public RectangularPrism createRectangularPrism(
			double lengthMeters, double widthMeters, double heightMeters) {
		int index = JSimJNI.createBody(worldHandle, 1.0);
		if (index < 0) {
			throw new JSimException("Failed to create rectangular prism in physics world", index,
				"The physics world may have reached its maximum capacity or internal resources are exhausted. Ensure the world is properly initialized.");
		}

		RectangularPrism prism = new RectangularPrism(
				this, index, lengthMeters, widthMeters, heightMeters);
		rectangularPrisms.add(prism);
		return prism;
	}

	/**
	 * Returns the balls created through this world wrapper in insertion order.
	 *
	 * @return immutable view of the created balls
	 */
	public List<Ball> balls() {
		return Collections.unmodifiableList(balls);
	}

	/**
	 * Returns the rectangular-prism hitboxes created through this world wrapper in insertion order.
	 *
	 * @return immutable view of the created rectangular-prism hitboxes
	 */
	public List<RectangularPrism> rectangularPrisms() {
		return Collections.unmodifiableList(rectangularPrisms);
	}

	/**
	 * Registers a callback that runs after each successful physics step.
	 *
	 * @param listener callback invoked after {@link #step()} completes
	 */
	public void addStepListener(Runnable listener) {
		if (listener != null) {
			stepListeners.add(listener);
		}
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
		int rc = JSimJNI.setBodyPosition(worldHandle, bodyIndex, xMeters, yMeters, zMeters);
		if (rc != 0) {
			throw new JSimException("Failed to set body position for bodyIndex=" + bodyIndex + " to (" + xMeters + ", " + yMeters + ", " + zMeters + ") m", rc,
				"Body index may be invalid or the world handle is corrupted. Check that bodyIndex >= 0 and the body exists in this world.");
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
		int rc = JSimJNI.setBodyLinearVelocity(worldHandle, bodyIndex, vxMetersPerSecond,
				vyMetersPerSecond, vzMetersPerSecond);
		if (rc != 0) {
			throw new JSimException("Failed to set body linear velocity for bodyIndex=" + bodyIndex + " to (" + vxMetersPerSecond + ", " + vyMetersPerSecond + ", " + vzMetersPerSecond + ") m/s", rc,
				"Body index may be invalid or velocity values are non-finite. Check that bodyIndex >= 0 and all velocities are finite numbers.");
		}
	}

	/**
	 * Enables or disables gravity for the given body.
	 *
	 * @param bodyIndex native body index
	 * @param enabled true to enable gravity
	 */
	void setBodyGravityEnabled(int bodyIndex, boolean enabled) {
		int rc = JSimJNI.setBodyGravityEnabled(worldHandle, bodyIndex, enabled);
		if (rc != 0) {
			throw new JSimException("Failed to set body gravity enabled=" + enabled + " for bodyIndex=" + bodyIndex, rc,
				"Body index may be invalid or does not exist. Check that bodyIndex >= 0 and refers to an existing body in this world.");
		}
	}

	/**
	 * Sets the body's collision filter for broad-phase collision detection.
	 * Bodies on the same collision layer will be filtered based on their respective
	 * collision masks, allowing fine-grained control over which bodies can collide.
	 *
	 * @param bodyIndex native body index
	 * @param collisionLayerBits collision layer bits (which group this body belongs to)
	 * @param collisionMaskBits collision mask bits (which groups this body will collide with)
	 */
	void setBodyCollisionFilter(int bodyIndex, int collisionLayerBits, int collisionMaskBits) {
		int rc = JSimJNI.setBodyCollisionFilter(worldHandle, bodyIndex, collisionLayerBits,
				collisionMaskBits);
		if (rc != 0) {
			throw new JSimException("Failed to set body collision filter for bodyIndex=" + bodyIndex + " with layerBits=" + collisionLayerBits + ", maskBits=" + collisionMaskBits, rc,
				"Body index may be invalid or does not exist. Check that bodyIndex >= 0 and refers to an existing body in this world.");
		}
	}

	/**
	 * Defines an approximate spherical collision/body shape for aerodynamic drag modeling.
	 * This sets both the collision geometry and aerodynamic drag calculations for the body.
	 *
	 * @param bodyIndex native body index
	 * @param radiusMeters sphere radius in meters
	 * @param dragCoefficient drag coefficient for aerodynamic calculations
	 */
	void setBodyAerodynamicSphere(int bodyIndex, double radiusMeters, double dragCoefficient) {
		int rc = JSimJNI.setBodyAerodynamicSphere(worldHandle, bodyIndex, radiusMeters,
				dragCoefficient);
		if (rc != 0) {
			throw new JSimException("Failed to set body aerodynamic sphere for bodyIndex=" + bodyIndex + " with radius=" + radiusMeters + " m, dragCoefficient=" + dragCoefficient, rc,
				"Body index may be invalid, or radius/drag coefficient are invalid. Radius must be positive and finite; drag coefficient must be non-negative and finite.");
		}
	}

	/**
	 * Defines an approximate box collision/body shape for aerodynamic drag modeling.
	 * This sets both the collision geometry and aerodynamic drag calculations for the body.
	 *
	 * @param bodyIndex native body index
	 * @param xMeters box x dimension in meters
	 * @param yMeters box y dimension in meters
	 * @param zMeters box z dimension in meters
	 * @param dragCoefficient drag coefficient for aerodynamic calculations
	 */
	void setBodyAerodynamicBox(int bodyIndex, double xMeters, double yMeters, double zMeters,
			double dragCoefficient) {
		int rc = JSimJNI.setBodyAerodynamicBox(worldHandle, bodyIndex, xMeters, yMeters,
				zMeters, dragCoefficient);
		if (rc != 0) {
			throw new JSimException("Failed to set body aerodynamic box for bodyIndex=" + bodyIndex + " with dimensions (" + xMeters + ", " + yMeters + ", " + zMeters + ") m, dragCoefficient=" + dragCoefficient, rc,
				"Body index may be invalid, or dimensions/drag coefficient are invalid. All dimensions must be positive and finite; drag coefficient must be non-negative and finite.");
		}
	}

	/**
	 * Sets the ball's world-space position in meters.
	 *
	 * @param ballIndex native ball index
	 * @param xMeters x position in meters
	 * @param yMeters y position in meters
	 * @param zMeters z position in meters
	 */
	void setBallPosition(int ballIndex, double xMeters, double yMeters, double zMeters) {
		int rc = JSimJNI.setBallPosition(worldHandle, ballIndex, xMeters, yMeters, zMeters);
		if (rc != 0) {
			throw new JSimException("Failed to set ball position", rc,
				"Verify the ball index and that the native world is valid; check native logs for details.");
		}
	}

	/**
	 * Sets the ball's world-space linear velocity in meters per second.
	 *
	 * @param ballIndex native ball index
	 * @param vxMetersPerSecond x velocity in meters per second
	 * @param vyMetersPerSecond y velocity in meters per second
	 * @param vzMetersPerSecond z velocity in meters per second
	 */
	void setBallLinearVelocity(int ballIndex, double vxMetersPerSecond, double vyMetersPerSecond,
			double vzMetersPerSecond) {
		int rc = JSimJNI.setBallLinearVelocity(worldHandle, ballIndex, vxMetersPerSecond,
				vyMetersPerSecond, vzMetersPerSecond);
		if (rc != 0) {
			throw new JSimException("Failed to set ball linear velocity", rc,
				"Verify the ball index and velocity parameters; check native logs for details.");
		}
	}

	/**
	 * Gets the world position for the given body.
	 *
	 * @param bodyIndex native body index
	 * @return body position
	 */
	public Pose3d getBodyPosition(int bodyIndex) {
		double[] values = new double[3];
		int rc = JSimJNI.getBodyPosition(worldHandle, bodyIndex, values);
		if (rc != 0) {
			throw new JSimException("Failed to get body position", rc,
				"Verify the body index and that the world is initialized; check native logs for details.");
		}
		return new Pose3d(values[0], values[1], values[2], new Rotation3d());
	}

	/**
	 * Gets the world linear velocity for the given body.
	 *
	 * @param bodyIndex native body index
	 * @return body linear velocity
	 */
	public LinearVelocity3d getBodyLinearVelocity(int bodyIndex) {
		double[] values = new double[3];
		int rc = JSimJNI.getBodyLinearVelocity(worldHandle, bodyIndex, values);
		if (rc != 0) {
			throw new JSimException("Failed to get body linear velocity", rc,
				"Verify the body index and that the world is initialized; check native logs for details.");
		}
		return new LinearVelocity3d(values[0], values[1], values[2]);
	}

	/**
	 * Exports full body state blocks.
	 *
	 * <p>Layout per body is: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz].
	 *
	 * @param outState13 destination array sized for N*13 entries
	 * @return number of body blocks written
	 */
	public int getBodyState13Array(double[] outState13) {
		int rc = JSimJNI.getBodyState13Array(worldHandle, outState13);
		if (rc < 0) {
			throw new JSimException("Failed to get body state array", rc,
				"Verify the destination array length and check native logs for details.");
		}
		return rc;
	}

	/**
	 * Gets the world position for the given ball.
	 *
	 * @param ballIndex native ball index
	 * @return ball position as a Pose3d with zero rotation
	 */
	public Pose3d getBallPosition(int ballIndex) {
		double[] values = new double[3];
		int rc = JSimJNI.getBallPosition(worldHandle, ballIndex, values);
		if (rc != 0) {
			throw new JSimException("Failed to get ball position", rc,
				"Verify the ball index and that the world is initialized; check native logs for details.");
		}
		return new Pose3d(values[0], values[1], values[2], new Rotation3d());
	}
	/**
	 * Gets the world linear velocity for the given ball.
	 *
	 * @param ballIndex native ball index
	 * @return ball linear velocity as a LinearVelocity3d
	 */
	public LinearVelocity3d getBallLinearVelocity(int ballIndex) {
		double[] values = new double[3];
		int rc = JSimJNI.getBallLinearVelocity(worldHandle, ballIndex, values);
		if (rc != 0) {
			throw new JSimException("Failed to get ball linear velocity for ballIndex=" + ballIndex, rc,
				"Ball index may be invalid or does not exist. Check that ballIndex >= 0 and refers to an existing ball in this world.");
		}
		return new LinearVelocity3d(values[0], values[1], values[2]);
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
		int rc = JSimJNI.stepWorld(worldHandle, steps);
		if (rc != 0) {
			throw new JSimException("Failed to step world by " + steps + " step(s)", rc,
				"World handle may be corrupted or previously destroyed. Ensure the PhysicsWorld has not been closed and the world handle is valid.");
		}
		for (Runnable listener : stepListeners) {
			listener.run();
		}
	}

	/**
	 * Applies a gravity vector to the world.
	 *
	 * @param gravity gravity vector in meters per second squared
	 */
	public void setGravity(Translation3d gravity) {
		setGravity(gravity.getX(), gravity.getY(), gravity.getZ());
	}

	/**
	 * Applies gravity components to the world.
	 *
	 * @param gxMetersPerSecondSquared x gravity in meters per second squared
	 * @param gyMetersPerSecondSquared y gravity in meters per second squared
	 * @param gzMetersPerSecondSquared z gravity in meters per second squared
	 */
	public void setGravity(double gxMetersPerSecondSquared, double gyMetersPerSecondSquared,
			double gzMetersPerSecondSquared) {
		int rc = JSimJNI.setWorldGravity(worldHandle, gxMetersPerSecondSquared,
				gyMetersPerSecondSquared, gzMetersPerSecondSquared);
		if (rc != 0) {
			throw new JSimException("Failed to set world gravity to (" + gxMetersPerSecondSquared + ", " + gyMetersPerSecondSquared + ", " + gzMetersPerSecondSquared + ") m/s^2", rc,
				"Gravity values must be finite numbers. Check that all gravity components are valid floating point values.");
		}
	}

	@Override
	public void close() {
		if (worldHandle != 0) {
			JSimJNI.destroyWorld(worldHandle);
			worldHandle = 0;
		}
	}
}
