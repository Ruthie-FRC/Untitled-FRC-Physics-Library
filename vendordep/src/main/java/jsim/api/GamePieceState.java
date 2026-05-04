package jsim.api;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Tracks the lifecycle and data of a standard game piece in the simulation.
 */
public class GamePieceState {

    /**
     * Enum representing the lifecycle state of a game piece.
     */
    public enum Lifecycle {
        /** Game piece has been spawned. */
        SPAWNED,
        /** Game piece is actively moving. */
        ACTIVE,
        /** Game piece is interacting with robot or field element. */
        INTERACTING,
        /** Game piece interaction has been resolved. */
        RESOLVED
    }

    /** The current lifecycle state of this game piece. */
    public Lifecycle lifecycle = Lifecycle.SPAWNED;
    /** The current 3D position of this game piece. */
    public Pose3d position = new Pose3d();
    /** The physics properties of this game piece. */
    public GamePiecePhysics physics;

    /**
     * Creates a new GamePieceState with the specified physics type.
     * @param physicsType The physics properties for this game piece.
     */
    public GamePieceState(GamePiecePhysics physicsType) {
        this.physics = physicsType;
    }

    /**
     * Defines valid interaction volume for intake.
     * @param intakeArea Array of Translation3d vertices defining the intake polygon in 3D space.
     */
    public void intakeZone(Translation3d[] intakeArea) {
        // Defines the polygon bounds in 3d space for intaking
    }

    /**
     * Ejects/spawns a piece based on parameters mapping.
     * @param exitAngle The rotation angle for the game piece exit.
     * @param exitVelocity The 3D velocity vector for the game piece exit.
     * @param robotOffsetStart The 2D offset from the robot start position.
     * @return A FieldState wrapper containing the spawned GamePieceState.
     */
    public static FieldState<GamePieceState> spawn(Rotation3d exitAngle, Translation3d exitVelocity, Translation2d robotOffsetStart) {
        GamePieceState state = new GamePieceState(new GamePiecePhysics());
        state.lifecycle = Lifecycle.ACTIVE;
        // In reality, this links back into the StateManager
        return new FieldState<>(state);
    }
}
