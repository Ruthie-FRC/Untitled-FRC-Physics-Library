package jsim.api;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import jsim.api.StateManager;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * External interface for simulated FRC Robots interacting with JSim.
 */
public class SimRobot {
    private final RobotID robotID;
    private final FieldState<RobotState> stateManagerRef;

    /**
     * Internal state for a simulated robot, including pose and speeds.
     */
    public static class RobotState {
        /**
         * The current field-relative pose of the robot.
         */
        public Pose2d pose = new Pose2d();
        /**
         * The current chassis speeds of the robot.
         */
        public ChassisSpeeds speeds = new ChassisSpeeds();
    }

    /**
     * Protected constructor. Use SimRobot.createRobot().
     * @param id The RobotID for this robot.
     * @param stateRef The field state reference for this robot.
     */
    protected SimRobot(RobotID id, FieldState<RobotState> stateRef) {
        this.robotID = id;
        this.stateManagerRef = stateRef;
    }

    /**
     * Initializes a new robot via the unified StateManager tracking system.
     * 
     * @param id The logical driver station id.
     * @param frameDimensions Translation2d[] for each vertex of the frame perimeter
     * @return the instantiated SimRobot handle.
     */
    public static SimRobot createRobot(RobotID id, Translation2d[] frameDimensions) {
        return StateManager.getInstance().initializeRobot(id, new Pose2d(), frameDimensions);
    }

    /**
     * Retrieves the current field-relative odometry pose. 
     * Pulled strictly from the StateManager snapshot.
     * @return the current pose of the robot.
     */
    public Pose2d getPose() {
        return stateManagerRef.get().pose;
    }

    /**
     * Hard overrides the state manager's simulation pose for this robot.
     * @param pose the new pose to set for the robot.
     */
    public void resetPose(Pose2d pose) {
        stateManagerRef.get().pose = pose;
    }

    /**
     * Applies commanded speeds to the physics solver.
     * @param speeds the chassis speeds to apply.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        stateManagerRef.get().speeds = speeds;
    }

    /**
     * Gets the RobotID for this robot.
     * @return the RobotID assigned to this robot.
     */
    public RobotID getRobotID() {
        return robotID;
    }
}
