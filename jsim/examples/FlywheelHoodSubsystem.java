package examples;

import api.GamepieceZone;
import api.SimRobot;
import api.Rotation3d;

/**
 * FlywheelHoodSubsystem simulates a flywheel shooter with a hood and a compression backspin roller.
 * This is a more advanced shooter that can impart backspin for higher arc shots.
 */
public class FlywheelHoodSubsystem {
  private final FlywheelSubsystem flywheel;
  private final GamepieceZone backspinRollerZone;
  private double backspinVelocity = 0.0;
  private Rotation3d exitAngle = new Rotation3d(0, 0, 0);

  /**
   * @param robot The simulated robot this subsystem is attached to.
   */
  public FlywheelHoodSubsystem(SimRobot robot) {
    this.flywheel = new FlywheelSubsystem(robot);
    this.backspinRollerZone = new GamepieceZone(robot);
    this.backspinRollerZone.setMode(GamepieceZone.Mode.DISABLED);
  }

  /**
   * Set the flywheel and backspin roller velocities and the exit angle.
   * @param left Left flywheel velocity
   * @param right Right flywheel velocity
   * @param backspin Backspin roller velocity
   * @param angle Exit angle for the shot
   */
  public void setShot(double left, double right, double backspin, Rotation3d angle) {
    flywheel.setFlywheel(left, right, angle);
    this.backspinVelocity = backspin;
    this.exitAngle = angle;
  }

  /**
   * Fire a game piece using the flywheel and backspin roller.
   * The flywheel provides the main exit speed, the roller imparts backspin.
   */
  public void shoot() {
    flywheel.shoot();
    backspinRollerZone.setExitParameters(backspinVelocity, exitAngle);
    backspinRollerZone.setMode(GamepieceZone.Mode.SHOOT);
  }

  /**
   * Stop the shooter and roller.
   */
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
