package examples;

import api.GamepieceZone;
import api.Rotation3d;
import api.SimRobot;

/**
 * Example that models a flywheel plus an adjustable hood for aiming.
 */
public class FlywheelHoodExample {
  private final FlywheelSubsystemExample flywheel;
  private final GamepieceZone hoodZone;
  private Rotation3d hoodAngle = new Rotation3d(0, 0, 0);

  public FlywheelHoodExample(SimRobot robot) {
    this.flywheel = new FlywheelSubsystemExample(robot);
    this.hoodZone = new GamepieceZone(robot);
    this.hoodZone.setMode(GamepieceZone.Mode.DISABLED);
  }

  public void setShot(double left, double right, Rotation3d angle) {
    flywheel.setFlywheel(left, right, angle);
    this.hoodAngle = angle;
  }

  public void fire() {
    flywheel.shoot();
    hoodZone.setExitParameters(0.0, hoodAngle);
    hoodZone.setMode(GamepieceZone.Mode.SHOOT);
  }

  public void stop() {
    flywheel.stop();
    hoodZone.setMode(GamepieceZone.Mode.DISABLED);
  }

  public FlywheelSubsystemExample getFlywheel() {
    return flywheel;
  }

  public GamepieceZone getHoodZone() {
    return hoodZone;
  }
}
