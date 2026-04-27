
package examples;

import api.SimRobot;
import api.GamepieceZone;
import driver.WPILibClones.Rotation3d;

public class ShooterSubsystem {
    private final GamepieceZone zone;
    private double exitVelocity = 10.0;
    private Rotation3d exitRotation = new Rotation3d(0, 0, 0);

    public ShooterSubsystem(SimRobot robot) {
        this.zone = new GamepieceZone(robot);
    }

    public void setShot(double velocity, Rotation3d rotation) {
        this.exitVelocity = velocity;
        this.exitRotation = rotation;
    }

    public void shoot() {
        zone.setExitParameters(exitVelocity, exitRotation);
        zone.setMode(GamepieceZone.Mode.SHOOT);
    }

    public void stop() {
        zone.setMode(GamepieceZone.Mode.DISABLED);
    }
}
