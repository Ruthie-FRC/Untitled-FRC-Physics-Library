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
 * Owns and mutates RobotState for simulation.
 * Single source of truth for robot motion state.
 */
public class RobotStateManager {

    private final Map<RobotID, FieldState<RobotState>> robotStates = new HashMap<>();

    /**
     * Registers a robot in the simulation.
     */
    public FieldState<RobotState> initializeRobot(
            RobotID id,
            Pose2d initialPose,
            Translation2d[] frameDimensions
    ) {
        RobotState state = new RobotState(id, initialPose, frameDimensions);
        FieldState<RobotState> fieldState = new FieldState<>(state);

        FieldState<RobotState> state =
            new FieldState<>(new RobotState(id, initialPose, frameDimensions));        
            robotStates.put(id, state);
        return state;
    }

    // =========================
    // POSE CONTROL
    // =========================

    public Pose2d getRobotPose(RobotID id) {
        return get(id).getState().pose;
    }

    public void resetRobotPose(RobotID id, Pose2d pose) {
        get(id).getState().pose = pose;
    }

    // =========================
    // CHASSIS SPEEDS
    // =========================

    public void setChassisSpeeds(RobotID id, ChassisSpeeds speeds) {
        get(id).getState().chassisSpeeds = speeds;
    }

    public ChassisSpeeds getChassisSpeeds(RobotID id) {
        return get(id).getState().chassisSpeeds;
    }

    // =========================
    // FIELD STATE ACCESS
    // =========================

    public FieldState<RobotState> getFieldState(RobotID id) {
        FieldState<RobotState> state = robotStates.get(id);
        if (state == null) {
            throw new IllegalStateException("Robot not initialized: " + id);
        }
        return state;
    }

    // =========================
    // INTERNAL
    // =========================

    private FieldState<RobotState> get(RobotID id) {
        FieldState<RobotState> state = robotStates.get(id);
        if (state == null) {
            throw new IllegalStateException("Robot not initialized: " + id);
        }
        return state;
    }
}
