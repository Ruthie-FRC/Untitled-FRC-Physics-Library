
package jsim.nt;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

/**
 * Unified pose estimator for a differential drive robot supporting both simple odometry and advanced estimation with vision.
 */
public class RobotPoseEstimator {
	private DifferentialDriveOdometry odometry = null;
	private DifferentialDrivePoseEstimator estimator = null;
	private boolean advanced = false;

	/**
	 * Simple odometry constructor.
	 */
	public RobotPoseEstimator(Pose2d initialPose, double initialGyroRadians) {
		this.odometry = new DifferentialDriveOdometry(
			new Rotation2d(initialGyroRadians),
			0.0, 0.0, initialPose
		);
		this.advanced = false;
	}

	/**
	 * Advanced estimator constructor.
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
	 * For simple odometry: (left, right, gyro)
	 * For advanced: (gyro, left, right, timestamp)
	 */
	public Pose2d update(double leftMeters, double rightMeters, double gyroRadians) {
		if (!advanced) {
			return odometry.update(new Rotation2d(gyroRadians), leftMeters, rightMeters);
		} else {
			throw new UnsupportedOperationException("Use update(gyro, left, right, timestamp) for advanced mode");
		}
	}

	public Pose2d update(double gyroRadians, double leftMeters, double rightMeters, double timestampSeconds) {
		if (advanced) {
			return estimator.updateWithTime(timestampSeconds, new Rotation2d(gyroRadians), leftMeters, rightMeters);
		} else {
			throw new UnsupportedOperationException("Use update(left, right, gyro) for simple mode");
		}
	}

	/**
	 * Adds a vision measurement (advanced mode only).
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
	 */
	public Pose2d getEstimatedPose() {
		if (advanced) {
			return estimator.getEstimatedPosition();
		} else {
			return odometry.getPoseMeters();
		}
	}
}
