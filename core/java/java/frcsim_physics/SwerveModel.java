package frcsim_physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import rensim.Vec3;

/**
 * Reduced-order swerve chassis model.
 *
 * <p>The model stores module locations in the chassis frame and maps per-module
 * drive forces into a chassis wrench using the baseline equations documented in
 * the repository docs. Steering angles are measured counterclockwise from the
 * chassis +X axis in the chassis frame.
 */
public final class SwerveModel {
	/**
	 * Per-module command/state used to build a chassis wrench.
	 *
	 * @param steeringAngleRadians steering angle in radians, measured from +X in the chassis frame
	 * @param driveForceNewtons drive force along the wheel forward axis in newtons
	 */
	public record ModuleState(double steeringAngleRadians, double driveForceNewtons) {
		public ModuleState {
			if (!Double.isFinite(steeringAngleRadians)) {
				throw new IllegalArgumentException("steeringAngleRadians must be finite");
			}
			if (!Double.isFinite(driveForceNewtons)) {
				throw new IllegalArgumentException("driveForceNewtons must be finite");
			}
		}
	}

	/**
	 * Resulting chassis wrench from the module forces.
	 *
	 * @param forceXNewtons net force in the chassis +X direction
	 * @param forceYNewtons net force in the chassis +Y direction
	 * @param yawTorqueNewtonMeters net yaw torque about the chassis +Z axis
	 */
	public record ChassisWrench(double forceXNewtons, double forceYNewtons, double yawTorqueNewtonMeters) {
		public ChassisWrench {
			if (!Double.isFinite(forceXNewtons) || !Double.isFinite(forceYNewtons)
					|| !Double.isFinite(yawTorqueNewtonMeters)) {
				throw new IllegalArgumentException("chassis wrench components must be finite");
			}
		}

		/**
		 * Converts the wrench to a vector where x/y are force and z is yaw torque.
		 *
		 * @return wrench encoded as a Vec3
		 */
		public Vec3 asVec3() {
			return new Vec3(forceXNewtons, forceYNewtons, yawTorqueNewtonMeters);
		}
	}

	/**
	 * Chassis acceleration derived from a wrench.
	 *
	 * @param accelXMps2 linear acceleration in the chassis +X direction
	 * @param accelYMps2 linear acceleration in the chassis +Y direction
	 * @param yawAccelRadps2 yaw angular acceleration about the chassis +Z axis
	 */
	public record ChassisAcceleration(double accelXMps2, double accelYMps2, double yawAccelRadps2) {
		public ChassisAcceleration {
			if (!Double.isFinite(accelXMps2) || !Double.isFinite(accelYMps2) || !Double.isFinite(yawAccelRadps2)) {
				throw new IllegalArgumentException("chassis acceleration components must be finite");
			}
		}
	}

	private final List<Vec3> modulePositionsMeters;
	private final double massKg;
	private final double yawMomentOfInertiaKgM2;

	/**
	 * Creates a swerve model from arbitrary module positions.
	 *
	 * @param modulePositionsMeters module positions in chassis coordinates
	 * @param massKg chassis mass in kilograms
	 * @param yawMomentOfInertiaKgM2 chassis yaw moment of inertia in kg*m^2
	 */
	public SwerveModel(List<Vec3> modulePositionsMeters, double massKg, double yawMomentOfInertiaKgM2) {
		if (modulePositionsMeters == null || modulePositionsMeters.isEmpty()) {
			throw new IllegalArgumentException("modulePositionsMeters must contain at least one module");
		}

		final List<Vec3> copiedPositions = new ArrayList<>(modulePositionsMeters.size());
		for (Vec3 positionMeters : modulePositionsMeters) {
			copiedPositions.add(Objects.requireNonNull(positionMeters, "module position cannot be null"));
		}

		this.modulePositionsMeters = List.copyOf(copiedPositions);
		this.massKg = massKg > 0.0 ? massKg : 1.0;
		this.yawMomentOfInertiaKgM2 = yawMomentOfInertiaKgM2 > 0.0 ? yawMomentOfInertiaKgM2 : 1.0;
	}

	/**
	 * Creates a square four-module swerve model.
	 *
	 * @param wheelBaseMeters distance between front and rear module centers
	 * @param trackWidthMeters distance between left and right module centers
	 * @param massKg chassis mass in kilograms
	 * @param yawMomentOfInertiaKgM2 chassis yaw moment of inertia in kg*m^2
	 * @return a four-module model laid out at the chassis corners
	 */
	public static SwerveModel square(double wheelBaseMeters, double trackWidthMeters, double massKg,
			double yawMomentOfInertiaKgM2) {
		final double halfWheelBase = Math.max(0.0, wheelBaseMeters) * 0.5;
		final double halfTrackWidth = Math.max(0.0, trackWidthMeters) * 0.5;

		return new SwerveModel(List.of(
				new Vec3(halfWheelBase, halfTrackWidth, 0.0),
				new Vec3(halfWheelBase, -halfTrackWidth, 0.0),
				new Vec3(-halfWheelBase, halfTrackWidth, 0.0),
				new Vec3(-halfWheelBase, -halfTrackWidth, 0.0)),
				massKg,
				yawMomentOfInertiaKgM2);
	}

	/**
	 * Returns the number of configured modules.
	 *
	 * @return module count
	 */
	public int moduleCount() {
		return modulePositionsMeters.size();
	}

	/**
	 * Returns the configured module positions.
	 *
	 * @return immutable module position list
	 */
	public List<Vec3> modulePositionsMeters() {
		return modulePositionsMeters;
	}

	/**
	 * Returns the chassis mass in kilograms.
	 *
	 * @return mass in kilograms
	 */
	public double massKg() {
		return massKg;
	}

	/**
	 * Returns the yaw moment of inertia used for angular acceleration estimates.
	 *
	 * @return yaw moment of inertia in kg*m^2
	 */
	public double yawMomentOfInertiaKgM2() {
		return yawMomentOfInertiaKgM2;
	}

	/**
	 * Computes the chassis wrench produced by the supplied module states.
	 *
	 * @param moduleStates per-module steering angle and drive force
	 * @return resulting chassis wrench
	 */
	public ChassisWrench computeChassisWrench(List<ModuleState> moduleStates) {
		validateModuleStates(moduleStates);

		double forceX = 0.0;
		double forceY = 0.0;
		double yawTorque = 0.0;

		for (int i = 0; i < moduleStates.size(); ++i) {
			final ModuleState moduleState = moduleStates.get(i);
			final Vec3 positionMeters = modulePositionsMeters.get(i);

			final double cos = Math.cos(moduleState.steeringAngleRadians());
			final double sin = Math.sin(moduleState.steeringAngleRadians());
			final double moduleForceX = moduleState.driveForceNewtons() * cos;
			final double moduleForceY = moduleState.driveForceNewtons() * sin;

			forceX += moduleForceX;
			forceY += moduleForceY;
			yawTorque += positionMeters.x() * moduleForceY - positionMeters.y() * moduleForceX;
		}

		return new ChassisWrench(forceX, forceY, yawTorque);
	}

	/**
	 * Computes the chassis acceleration implied by the supplied module states.
	 *
	 * @param moduleStates per-module steering angle and drive force
	 * @return resulting chassis acceleration
	 */
	public ChassisAcceleration computeChassisAcceleration(List<ModuleState> moduleStates) {
		final ChassisWrench wrench = computeChassisWrench(moduleStates);
		return new ChassisAcceleration(
				wrench.forceXNewtons() / massKg,
				wrench.forceYNewtons() / massKg,
				wrench.yawTorqueNewtonMeters() / yawMomentOfInertiaKgM2);
	}

	private void validateModuleStates(List<ModuleState> moduleStates) {
		if (moduleStates == null) {
			throw new IllegalArgumentException("moduleStates cannot be null");
		}
		if (moduleStates.size() != modulePositionsMeters.size()) {
			throw new IllegalArgumentException(
					"Expected " + modulePositionsMeters.size() + " module states but received " + moduleStates.size());
		}
		for (ModuleState moduleState : moduleStates) {
			Objects.requireNonNull(moduleState, "module state cannot be null");
		}
	}
}
