package jsim.nt;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * Advanced pose estimator for a differential drive robot using encoders, gyro, and vision measurements.
 */
public class AdvancedRobotPoseEstimator {
    private final DifferentialDrivePoseEstimator estimator;

    /**
     * Constructs an AdvancedRobotPoseEstimator.
     *
     * @param kinematics DifferentialDriveKinematics instance for your robot
     * @param initialPose Initial robot pose
     * @param initialGyroRadians Initial gyro angle (radians, CCW+)
     * @param stateStdDevs Standard deviations for [x, y, theta] process noise
     * @param localMeasurementStdDevs Standard deviations for [left, right, gyro] local sensors
     * @param visionMeasurementStdDevs Standard deviations for [x, y, theta] vision measurements
     */
    public AdvancedRobotPoseEstimator(
            DifferentialDriveKinematics kinematics,
            Pose2d initialPose,
            double initialGyroRadians,
            double[] stateStdDevs,
            double[] localMeasurementStdDevs,
            double[] visionMeasurementStdDevs) {
        this.estimator = new DifferentialDrivePoseEstimator(
            kinematics,
            new Rotation2d(initialGyroRadians),
            0.0, // initial left encoder
            0.0, // initial right encoder
            initialPose
        );
    }

    /**
     * Updates the pose estimator with new sensor readings.
     *
     * @param gyroRadians Current gyro angle (radians, CCW+)
     * @param leftMeters Total left encoder distance (meters)
     * @param rightMeters Total right encoder distance (meters)
     * @param timestampSeconds Timestamp of the measurement (seconds)
     * @return Estimated robot pose
     */
    public Pose2d update(double gyroRadians, double leftMeters, double rightMeters, double timestampSeconds) {
        return estimator.updateWithTime(
            timestampSeconds,
            new Rotation2d(gyroRadians),
            leftMeters,
            rightMeters
        );
    }

    /**
     * Adds a vision measurement to the estimator.
     *
     * @param visionPose The pose measured by vision
     * @param timestampSeconds The timestamp of the vision measurement
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        estimator.addVisionMeasurement(visionPose, timestampSeconds);
    }

    /**
     * Gets the current estimated pose.
     *
     * @return The current estimated pose
     */
    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPosition();
    }
}
