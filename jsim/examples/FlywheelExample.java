package examples;

import api.Rotation3d;
import api.SimRobot;

/**
 * Simple example wrapper around a generic FlywheelSubsystem.
 */
public class FlywheelExample {
  private final FlywheelSubsystem flywheel;

  public FlywheelExample(SimRobot robot) {
    this.flywheel = new FlywheelSubsystem(robot);
  }

  public void set(double left, double right, Rotation3d angle) {
    flywheel.setFlywheel(left, right, angle);
  }

  public void fire() {
    flywheel.shoot();
  }

  public void stop() {
    flywheel.stop();
  }

  public FlywheelSubsystem getFlywheel() {
    return flywheel;
  }
}
