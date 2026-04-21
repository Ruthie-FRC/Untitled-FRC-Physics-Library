
package jsim.nt;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;


/**
 * <p>
 * Unified pose estimator for a differential drive robot. Supports both simple odometry (encoder + gyro) and advanced pose estimation with vision integration.
 * </p>
 * <p>
 * <b>Usage:</b>
 * <ul>
 *   <li>For simple odometry, use the {@link #RobotPoseEstimator(Pose2d, double)} constructor and call {@link #update(double, double, double)}.</li>
 *   <li>For advanced estimation (with vision), use the {@link #RobotPoseEstimator(DifferentialDriveKinematics, Pose2d, double, double[], double[], double[])} constructor and call {@link #update(double, double, double, double)} and {@link #addVisionMeasurement(Pose2d, double)}.</li>
 * </ul>
 * </p>
 * <p>
 * <b>Note:</b> This class is a thin wrapper around WPILib's DifferentialDriveOdometry and DifferentialDrivePoseEstimator.
 * </p>
 */
public class RobotPoseEstimator {
	/** Simple odometry estimator (null if advanced mode). */
	private DifferentialDriveOdometry odometry = null;
	/** Advanced estimator with vision (null if simple mode). */
	private DifferentialDrivePoseEstimator estimator = null;
	/** True if using advanced estimator. */
	private boolean advanced = false;

	/**
	 * Constructs a simple odometry estimator (no vision integration).
	 *
	 * @param initialPose Initial robot pose on the field.
	 * @param initialGyroRadians Initial gyro angle in radians (field-relative, CCW+).
	 */
	public RobotPoseEstimator(Pose2d initialPose, double initialGyroRadians) {
		this.odometry = new DifferentialDriveOdometry(
				new Rotation2d(initialGyroRadians),
				0.0, 0.0, initialPose
		);
		this.advanced = false;
	}

	/**
	 * Constructs an advanced estimator with vision integration.
	 *
	 * @param kinematics Differential drive kinematics (track width, etc).
	 * @param initialPose Initial robot pose on the field.
	 * @param initialGyroRadians Initial gyro angle in radians (field-relative, CCW+).
	 * @param stateStdDevs Standard deviations for state estimate (x, y, theta).
	 * @param localMeasurementStdDevs Standard deviations for local sensors (encoders, gyro).
	 * @param visionMeasurementStdDevs Standard deviations for vision measurements (x, y, theta).
	 */
	public RobotPoseEstimator(
			DifferentialDriveKinematics kinematics,
			Pose2d initialPose,
			double initialGyroRadians,
			double[] stateStdDevs,
			double[] localMeasurementStdDevs,
			double[] visionMeasurementStdDevs) {
		this.estimator = new DifferentialDrivePoseEstimator(
				kinematics,
				new Rotation2d(initialGyroRadians),
				0.0, 0.0, initialPose
		);
		this.advanced = true;
	}

	/**
	 * Updates the estimator with new sensor readings.
	 * <p>
	 * <b>Simple odometry mode:</b> Call with (left, right, gyro) values.
	 * <b>Advanced mode:</b> Throws exception; use {@link #update(double, double, double, double)} instead.
	 * </p>
	 *
	 * @param leftMeters Left encoder position (meters).
	 * @param rightMeters Right encoder position (meters).
	 * @param gyroRadians Gyro angle (radians, field-relative, CCW+).
	 * @return Estimated robot pose.
	 * @throws UnsupportedOperationException if called in advanced mode.
	 */
	public Pose2d update(double leftMeters, double rightMeters, double gyroRadians) {
		if (!advanced) {
			return odometry.update(new Rotation2d(gyroRadians), leftMeters, rightMeters);
		} else {
			throw new UnsupportedOperationException("Use update(gyro, left, right, timestamp) for advanced mode");
		}
	}

	/**
	 * Updates the estimator with new sensor readings and timestamp (advanced mode only).
	 * <p>
	 * <b>Advanced mode:</b> Call with (gyro, left, right, timestamp) values.
	 * <b>Simple odometry mode:</b> Throws exception; use {@link #update(double, double, double)} instead.
	 * </p>
	 *
	 * @param gyroRadians Gyro angle (radians, field-relative, CCW+).
	 * @param leftMeters Left encoder position (meters).
	 * @param rightMeters Right encoder position (meters).
	 * @param timestampSeconds Timestamp (seconds).
	 * @return Estimated robot pose.
	 * @throws UnsupportedOperationException if called in simple mode.
	 */
	public Pose2d update(double gyroRadians, double leftMeters, double rightMeters, double timestampSeconds) {
		if (advanced) {
			return estimator.updateWithTime(timestampSeconds, new Rotation2d(gyroRadians), leftMeters, rightMeters);
		} else {
			throw new UnsupportedOperationException("Use update(left, right, gyro) for simple mode");
		}
	}

	/**
	 * Adds a vision measurement to the estimator (advanced mode only).
	 *
	 * @param visionPose Vision-estimated robot pose.
	 * @param timestampSeconds Timestamp of vision measurement (seconds).
	 * @throws UnsupportedOperationException if called in simple mode.
	 */
	public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
		if (advanced) {
			estimator.addVisionMeasurement(visionPose, timestampSeconds);
		} else {
			throw new UnsupportedOperationException("Vision measurements only supported in advanced mode");
		}
	}

	/**
	 * Gets the current estimated pose.
	 *
	 * @return Current estimated robot pose.
	 */
	public Pose2d getEstimatedPose() {
		if (advanced) {
			return estimator.getEstimatedPosition();
		} else {
			return odometry.getPoseMeters();
		}
	}
}
