
package jsim.nt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for RobotPoseEstimator.
 */
public class RobotPoseEstimatorTest {
	@Test
	void testSimpleOdometryUpdate() {
		Pose2d initialPose = new Pose2d();
		double initialGyro = 0.0;
		RobotPoseEstimator estimator = new RobotPoseEstimator(initialPose, initialGyro);
		Pose2d pose = estimator.update(1.0, 1.0, 0.0);
		assertNotNull(pose);
	}

	@Test
	void testAdvancedEstimatorUpdate() {
		DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6);
		Pose2d initialPose = new Pose2d();
		double initialGyro = 0.0;
		double[] stateStdDevs = {0.1, 0.1, 0.1};
		double[] localStdDevs = {0.1, 0.1, 0.1};
		double[] visionStdDevs = {0.1, 0.1, 0.1};
		RobotPoseEstimator estimator = new RobotPoseEstimator(
				kinematics, initialPose, initialGyro, stateStdDevs, localStdDevs, visionStdDevs);
		Pose2d pose = estimator.update(0.0, 1.0, 1.0, 0.02);
		assertNotNull(pose);
	}

	@Test
	void testVisionMeasurementThrowsInSimpleMode() {
		RobotPoseEstimator estimator = new RobotPoseEstimator(new Pose2d(), 0.0);
		assertThrows(UnsupportedOperationException.class, () ->
				estimator.addVisionMeasurement(new Pose2d(), 0.0));
	}
}
