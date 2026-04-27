package api;

import driver.WPILibClones.Rotation3d;
import driver.WPILibClones.Translation2d;
import driver.WPILibClones.Translation3d;

public class GamePieceState {
    private GamePieceType type;
    private double velocity;
    private Rotation3d rotation;
    private Translation2d robotOffsetStart;
    private Translation3d exitVelocity;
    private Rotation3d exitAngle;
    private Translation3d[] intakeArea;

    public GamePieceState(GamePieceType type) {
        this.type = type;
    }

    public GamePieceType getType() {
        return type;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public void setRotation(Rotation3d rotation) {
        this.rotation = rotation;
    }

    public double getVelocity() {
        return velocity;
    }

    public Rotation3d getRotation() {
        return rotation;
    }

    public void setRobotOffsetStart(Translation2d offset) {
        this.robotOffsetStart = offset;
    }

    public Translation2d getRobotOffsetStart() {
        return robotOffsetStart;
    }

    public void setExitVelocity(Translation3d velocity) {
        this.exitVelocity = velocity;
    }

    public Translation3d getExitVelocity() {
        return exitVelocity;
    }

    public void setExitAngle(Rotation3d angle) {
        this.exitAngle = angle;
    }

    public Rotation3d getExitAngle() {
        return exitAngle;
    }

    public void intakeZone(Translation3d[] intakeArea) {
        this.intakeArea = intakeArea;
    }

    public Translation3d[] getIntakeArea() {
        return intakeArea;
    }

    // Children classes for specific game pieces
    public static class Fuel2026 extends GamePieceState {
        public Fuel2026() { super(GamePieceType.FUEL); }
        public void shoot(driver.WPILibClones.Translation3d relativeStart, double timeOfFlight, driver.WPILibClones.Rotation3d exitAngle) {
            // Minimal placeholder: record exit angle and velocity magnitude/time if needed.
            setExitAngle(exitAngle);
            // TODO: implement domain-specific prediction if required
        }
    }

    public static class Coral2025 extends GamePieceState {
        public Coral2025() { super(GamePieceType.CORAL); }
        public void place(driver.WPILibClones.Pose3d branch) {
            // TODO: Implement placement logic — placeholder to satisfy API surface.
        }
    }
}
