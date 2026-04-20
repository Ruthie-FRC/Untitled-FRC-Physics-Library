package jsim.nt;

import jsim.PhysicsWorld;
import jsim.PhysicsBody;
import jsim.Vec3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Example usage of RobotPoseEstimator in a JSim simulation loop.
 */
public class RobotPoseEstimatorExample {
    /**
     * Runs the RobotPoseEstimator simulation example.
     *
     * @param args Command-line arguments (not used)
     */
    public static void main(String[] args) {
        // Create a physics world with 20ms timestep and gravity enabled
        PhysicsWorld world = new PhysicsWorld(0.02, true);
        PhysicsBody robot = world.createBody(50.0); // 50kg robot

        // Initial pose and gyro
        Pose2d initialPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        double initialGyro = 0.0;
        RobotPoseEstimator estimator = new RobotPoseEstimator(initialPose, initialGyro);

        // Simulated encoder and gyro values
        double leftEncoder = 0.0;
        double rightEncoder = 0.0;
        double gyro = 0.0;
        double wheelBase = 0.6; // meters
        double wheelDelta = 0.05; // meters per step

        // Simulate 100 steps of forward motion
        for (int i = 0; i < 100; i++) {
            // Simulate encoders (both wheels move forward)
            leftEncoder += wheelDelta;
            rightEncoder += wheelDelta;
            // Simulate gyro (no rotation)
            gyro = 0.0;

            // Update estimator
            Pose2d estimatedPose = estimator.update(leftEncoder, rightEncoder, gyro);

            // Optionally, update the robot's true position in the world (for ground truth)
            robot.setPosition(new Vec3(estimatedPose.getX(), estimatedPose.getY(), 0.0));

            // Print estimated pose
            System.out.printf("Step %d: Estimated Pose: x=%.3f, y=%.3f, theta=%.3f\n",
                i, estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getRadians());

            // Advance simulation
            world.step();
        }

        world.close();
    }
}
