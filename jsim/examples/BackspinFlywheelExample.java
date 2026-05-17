package examples;

import api.GamepieceZone;
import api.Rotation3d;
import api.SimRobot;

/**
 * Example that adds a compression/backspin roller to a generic flywheel.
 */
public class BackspinFlywheelExample {
  private final FlywheelSubsystem flywheel;
  private final GamepieceZone backspinRollerZone;
  private double backspinVelocity = 0.0;
  private Rotation3d exitAngle = new Rotation3d(0, 0, 0);

  public BackspinFlywheelExample(SimRobot robot) {
    this.flywheel = new FlywheelSubsystem(robot);
    this.backspinRollerZone = new GamepieceZone(robot);
    this.backspinRollerZone.setMode(GamepieceZone.Mode.DISABLED);
  }

  public void setShot(double left, double right, double backspin, Rotation3d angle) {
    flywheel.setFlywheel(left, right, angle);
    this.backspinVelocity = backspin;
    this.exitAngle = angle;
  }

  public void fire() {
    flywheel.shoot();
    backspinRollerZone.setExitParameters(backspinVelocity, exitAngle);
    backspinRollerZone.setMode(GamepieceZone.Mode.SHOOT);
  }

  public void stop() {
    flywheel.stop();
    backspinRollerZone.setMode(GamepieceZone.Mode.DISABLED);
  }

  public FlywheelSubsystem getFlywheel() {
    return flywheel;
  }

  public GamepieceZone getBackspinRollerZone() {
    return backspinRollerZone;
  }
}
