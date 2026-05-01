package frc.robot.sim.core;

import api.ChassisSpeeds;
import api.FieldState;
import api.Pose2d;
import api.RobotID;
import api.RobotState;
import api.Translation2d;

import java.util.HashMap;
import java.util.Map;

/**
 * Handles all robot-related simulation state:
 * - Pose
 * - Chassis speeds
 * - FieldState reference
 *
 * Separate from game piece logic (StateManager).
 */
public class RobotStateManager {

    private final Map<RobotID, Pose2d> robotPoses = new HashMap<>();
    private final Map<RobotID, ChassisSpeeds> robotSpeeds = new HashMap<>();
    private final Map<RobotID, FieldState<RobotState>> robotStates = new HashMap<>();

    /**
     * Registers a robot in the simulation.
     */
    public FieldState<RobotState> initializeRobot(
            RobotID id,
            Pose2d initialPose,
            Translation2d[] frameDimensions
    ) {
        robotPoses.put(id, initialPose);
        robotSpeeds.put(id, new ChassisSpeeds(0, 0, 0));

        // You can expand this if your FieldState actually needs frameDimensions
        FieldState<RobotState> state =
            new FieldState<>(new RobotState(id, initialPose, frameDimensions));        
            robotStates.put(id, state);
        return state;
    }

    /**
     * Returns the current pose of the robot.
     */
    public Pose2d getRobotPose(RobotID id) {
        Pose2d pose = robotPoses.get(id);
        if (pose == null) {
            throw new IllegalStateException("Robot not initialized: " + id);
        }
        return pose;
    }

    /**
     * Hard reset of robot pose.
     */
    public void resetRobotPose(RobotID id, Pose2d pose) {
        if (!robotPoses.containsKey(id)) {
            throw new IllegalStateException("Robot not initialized: " + id);
        }
        robotPoses.put(id, pose);
    }

    /**
     * Sets chassis speeds (vx, vy, omega).
     */
    public void setChassisSpeeds(RobotID id, ChassisSpeeds speeds) {
        if (!robotSpeeds.containsKey(id)) {
            throw new IllegalStateException("Robot not initialized: " + id);
        }
        robotSpeeds.put(id, speeds);
    }

    /**
     * Optional: expose speeds if your sim loop needs them.
     */
    public ChassisSpeeds getChassisSpeeds(RobotID id) {
        ChassisSpeeds speeds = robotSpeeds.get(id);
        if (speeds == null) {
            throw new IllegalStateException("Robot not initialized: " + id);
        }
        return speeds;
    }

    /**
     * Access to the FieldState reference.
     */
    public FieldState<RobotState> getFieldState(RobotID id) {
        FieldState<RobotState> state = robotStates.get(id);
        if (state == null) {
            throw new IllegalStateException("Robot not initialized: " + id);
        }
        return state;
    }
}