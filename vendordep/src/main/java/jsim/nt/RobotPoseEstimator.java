package jsim.nt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

/**
 * Simple pose estimator for a differential drive robot using encoder and gyro data.
 * Extend for more advanced estimation or other drive types.
 */
public class RobotPoseEstimator {
    private final DifferentialDriveOdometry odometry;

    /**
     * Constructs a RobotPoseEstimator.
     *
     * @param initialPose Initial robot pose
     * @param initialGyroRadians Initial gyro angle (radians, CCW+)
     */
    public RobotPoseEstimator(Pose2d initialPose, double initialGyroRadians) {
        this.odometry = new DifferentialDriveOdometry(
            new Rotation2d(initialGyroRadians),
            0.0, // initial left encoder distance
            0.0, // initial right encoder distance
            initialPose
        );
    }

    /**
     * Updates the pose estimator with new sensor readings.
     *
     * @param leftMeters Total left encoder distance (meters)
     * @param rightMeters Total right encoder distance (meters)
     * @param gyroRadians Current gyro angle (radians, CCW+)
     * @return Estimated robot pose
     */
    public Pose2d update(double leftMeters, double rightMeters, double gyroRadians) {
        return odometry.update(new Rotation2d(gyroRadians), leftMeters, rightMeters);
    }

    /**
     * Gets the current estimated pose.
     *
     * @return The current estimated pose
     */
    public Pose2d getEstimatedPose() {
        return odometry.getPoseMeters();
    }
}
