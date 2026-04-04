package frcsim_physics;

import java.util.Arrays;
import java.util.Objects;
import rensim.Vec3;

/**
 * Simple rigid-body swerve kinematics model for simulation command generation.
 */
public final class SwerveModel {
	/**
	 * Chassis velocity command in robot frame.
	 *
	 * @param vxMps x linear speed in m/s
	 * @param vyMps y linear speed in m/s
	 * @param omegaRadPerSec yaw rate in rad/s
	 */
	public record ChassisCommand(double vxMps, double vyMps, double omegaRadPerSec) {}

	/**
	 * Wheel state command.
	 *
	 * @param speedMps wheel speed in m/s
	 * @param angleRad wheel steer angle in radians
	 */
	public record WheelCommand(double speedMps, double angleRad) {}

	/**
	 * Ordered wheel positions for state arrays.
	 */
	public enum WheelIndex {
		FRONT_LEFT,
		FRONT_RIGHT,
		REAR_LEFT,
		REAR_RIGHT
	}

	private final double wheelBaseMeters;
	private final double trackWidthMeters;
	private final double maxWheelSpeedMps;

	/**
	 * Creates a swerve kinematic model.
	 *
	 * @param wheelBaseMeters front-to-back wheel spacing
	 * @param trackWidthMeters left-to-right wheel spacing
	 * @param maxWheelSpeedMps maximum allowed wheel speed
	 */
	public SwerveModel(double wheelBaseMeters, double trackWidthMeters, double maxWheelSpeedMps) {
		if (!(wheelBaseMeters > 0.0)) {
			throw new IllegalArgumentException("wheelBaseMeters must be > 0");
		}
		if (!(trackWidthMeters > 0.0)) {
			throw new IllegalArgumentException("trackWidthMeters must be > 0");
		}
		if (!(maxWheelSpeedMps > 0.0)) {
			throw new IllegalArgumentException("maxWheelSpeedMps must be > 0");
		}
		this.wheelBaseMeters = wheelBaseMeters;
		this.trackWidthMeters = trackWidthMeters;
		this.maxWheelSpeedMps = maxWheelSpeedMps;
	}

	/**
	 * Computes per-wheel steer/speed commands from chassis motion.
	 *
	 * @param command desired chassis velocity command
	 * @return array of wheel commands in {@link WheelIndex} order
	 */
	public WheelCommand[] computeWheelCommands(ChassisCommand command) {
		Objects.requireNonNull(command);

		double halfWheelBase = wheelBaseMeters * 0.5;
		double halfTrackWidth = trackWidthMeters * 0.5;

		double a = command.vxMps - (command.omegaRadPerSec * halfWheelBase);
		double b = command.vxMps + (command.omegaRadPerSec * halfWheelBase);
		double c = command.vyMps - (command.omegaRadPerSec * halfTrackWidth);
		double d = command.vyMps + (command.omegaRadPerSec * halfTrackWidth);

		WheelCommand[] wheels = new WheelCommand[WheelIndex.values().length];
		wheels[WheelIndex.FRONT_LEFT.ordinal()] = wheelFromVector(b, d);
		wheels[WheelIndex.FRONT_RIGHT.ordinal()] = wheelFromVector(b, c);
		wheels[WheelIndex.REAR_LEFT.ordinal()] = wheelFromVector(a, d);
		wheels[WheelIndex.REAR_RIGHT.ordinal()] = wheelFromVector(a, c);

		normalizeWheelSpeeds(wheels);
		return wheels;
	}

	/**
	 * Applies a chassis command directly to a rigid body for high-level simulation.
	 *
	 * <p>Command is interpreted in world frame for simplicity.
	 *
	 * @param body rigid body to command
	 * @param command desired chassis velocity command
	 */
	public void applyToBody(RigidBody body, ChassisCommand command) {
		Objects.requireNonNull(body);
		Objects.requireNonNull(command);

		WheelCommand[] wheels = computeWheelCommands(command);
		double averageSpeed = Arrays.stream(wheels).mapToDouble(WheelCommand::speedMps).average().orElse(0.0);
		if (averageSpeed < 1.0e-9) {
			body.setLinearVelocityMps(Vec3.ZERO);
			body.setAngularVelocityRadPerSec(new Vec3(0.0, 0.0, command.omegaRadPerSec));
			return;
		}

		body.setLinearVelocityMps(new Vec3(command.vxMps, command.vyMps, 0.0));
		body.setAngularVelocityRadPerSec(new Vec3(0.0, 0.0, command.omegaRadPerSec));
	}

	private WheelCommand wheelFromVector(double vx, double vy) {
		double speed = Math.hypot(vx, vy);
		double angle = Math.atan2(vy, vx);
		return new WheelCommand(speed, angle);
	}

	private void normalizeWheelSpeeds(WheelCommand[] wheels) {
		double maxObserved = 0.0;
		for (WheelCommand wheel : wheels) {
			maxObserved = Math.max(maxObserved, wheel.speedMps);
		}

		if (maxObserved <= maxWheelSpeedMps) {
			return;
		}

		double scale = maxWheelSpeedMps / maxObserved;
		for (int i = 0; i < wheels.length; i++) {
			WheelCommand wheel = wheels[i];
			wheels[i] = new WheelCommand(wheel.speedMps * scale, wheel.angleRad);
		}
	}
}
