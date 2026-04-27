package jsim.api;

import jsim.driver.WPILibClones.Translation2d;
import jsim.driver.WPILibClones.Pose2d;
import jsim.driver.WPILibClones.ChassisSpeeds;
import jsim.api.RobotID;
import jsim.api.FieldState;
import jsim.core.StateManager;

/**
 * Minimal simulation-side robot representation used by example code.
 * The createRobot factory mirrors the usage in examples and stores a simple footprint.
 */
public final class SimRobot {
    private final RobotID robotID;
    private final StateManager stateManager;
    private final FieldState<RobotState> stateManagerRef;

    private SimRobot(RobotID robotID, StateManager stateManager, FieldState<RobotState> stateManagerRef) {
        this.robotID = robotID;
        this.stateManager = stateManager;
        this.stateManagerRef = stateManagerRef;
    }

    public static SimRobot createRobot(Translation2d[] frameDimensions, StateManager stateManager, RobotID robotID) {
        FieldState<RobotState> ref = stateManager.initializeRobot(robotID, new Pose2d(0,0,0), frameDimensions);
        return new SimRobot(robotID, stateManager, ref);
    }

    public Pose2d getPose() {
        return stateManager.getRobotPose(robotID);
    }

    public RobotID getRobotID() {
        return robotID;
    }

    public void resetPose(Pose2d pose) {
        stateManager.resetRobotPose(robotID, pose);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        stateManager.setChassisSpeeds(robotID, speeds);
    }

    public FieldState<RobotState> getStateManagerRef() {
        return stateManagerRef;
    }
}
