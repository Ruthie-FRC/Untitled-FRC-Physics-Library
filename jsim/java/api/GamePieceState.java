package api;

import api.Rotation3d;
import api.Translation2d;
import api.Translation3d;

/**
 * Represents the state of a game piece in the simulation.
 * Includes type, velocity, rotation, and other physical properties.
 */
public class GamePieceState {
    private GamePieceType type;
    private double velocity;
    private Rotation3d rotation;
    private Translation2d robotOffsetStart;
    private Translation3d exitVelocity;
    private Rotation3d exitAngle;
    private Translation3d[] intakeArea;


    /**
     * Constructs a new GamePieceState with the specified type.
     * @param type The type of the game piece.
     */
    public GamePieceState(GamePieceType type) {
        this.type = type;
    }


    /**
     * Gets the type of the game piece.
     * @return The game piece type.
     */
    public GamePieceType getType() {
        return type;
    }


    /**
     * Sets the velocity of the game piece.
     * @param velocity The velocity value.
     */
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }


    /**
     * Sets the rotation of the game piece.
     * @param rotation The rotation value.
     */
    public void setRotation(Rotation3d rotation) {
        this.rotation = rotation;
    }


    /**
     * Gets the velocity of the game piece.
     * @return The velocity value.
     */
    public double getVelocity() {
        return velocity;
    }


    /**
     * Gets the rotation of the game piece.
     * @return The rotation value.
     */
    public Rotation3d getRotation() {
        return rotation;
    }


    /**
     * Sets the robot offset at the start.
     * @param offset The offset value.
     */
    public void setRobotOffsetStart(Translation2d offset) {
        this.robotOffsetStart = offset;
    }


    /**
     * Gets the robot offset at the start.
     * @return The offset value.
     */
    public Translation2d getRobotOffsetStart() {
        return robotOffsetStart;
    }


    /**
     * Sets the exit velocity of the game piece.
     * @param velocity The exit velocity value.
     */
    public void setExitVelocity(Translation3d velocity) {
        this.exitVelocity = velocity;
    }


    /**
     * Gets the exit velocity of the game piece.
     * @return The exit velocity value.
     */
    public Translation3d getExitVelocity() {
        return exitVelocity;
    }


    /**
     * Sets the exit angle of the game piece.
     * @param angle The exit angle value.
     */
    public void setExitAngle(Rotation3d angle) {
        this.exitAngle = angle;
    }


    /**
     * Gets the exit angle of the game piece.
     * @return The exit angle value.
     */
    public Rotation3d getExitAngle() {
        return exitAngle;
    }


    /**
     * Sets the intake zone area for the game piece.
     * @param intakeArea The intake area array.
     */
    public void intakeZone(Translation3d[] intakeArea) {
        this.intakeArea = intakeArea;
    }


    /**
     * Gets the intake zone area for the game piece.
     * @return The intake area array.
     */
    public Translation3d[] getIntakeArea() {
        return intakeArea;
    }

    // Children classes for specific game pieces

    // Fuel2026 class moved to its own file.

    // Coral2025 class moved to its own file.
}
