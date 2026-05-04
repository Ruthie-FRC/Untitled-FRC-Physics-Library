package examples;

import api.GamepieceZone;
import api.SimRobot;
import api.Rotation3d;

/**
 * A subsystem representing a shooter mechanism on a simulated robot.
 *
 * <p>This class encapsulates a {@link GamepieceZone} to control the release of a game piece,
 * simulating a flywheel shooter or similar mechanism.
 */
public class ShooterSubsystem {
  private final GamepieceZone zone;
  private double exitVelocity = 10.0; // Default exit velocity in m/s
  private Rotation3d exitRotation = new Rotation3d(0, 0, 0); // Default exit angle

  /**
   * Constructs a new ShooterSubsystem.
   *
   * @param robot The {@link SimRobot} this subsystem is a part of. The GamepieceZone will be
   *     associated with this robot.
   */
  public ShooterSubsystem(SimRobot robot) {
    this.zone = new GamepieceZone(robot);
    // It's good practice to start in a disabled state.
    this.zone.setMode(GamepieceZone.Mode.DISABLED);
  }

  /**
   * Configures the parameters for the next shot.
   *
   * @param velocity The velocity (in meters per second) at which the game piece will be launched.
   * @param rotation The exit angle of the game piece relative to the robot's orientation.
   */
  public void setShot(double velocity, Rotation3d rotation) {
    this.exitVelocity = velocity;
    this.exitRotation = rotation;
  }

  /**
   * Activates the shooter to launch a game piece with the configured parameters.
   *
   * <p>This sets the GamepieceZone mode to SHOOT, which will release a game piece on the next
   * simulation step if one is available.
   */
  public void shoot() {
    zone.setExitParameters(exitVelocity, exitRotation);
    zone.setMode(GamepieceZone.Mode.SHOOT);
  }

  /**
   * Deactivates the shooter.
   *
   * <p>This sets the GamepieceZone mode to DISABLED, preventing it from launching any game pieces.
   */
  public void stop() {
    zone.setMode(GamepieceZone.Mode.DISABLED);
  }

  /**
   * Gets the underlying GamepieceZone instance.
   *
   * @return The GamepieceZone used by this subsystem.
   */
  public GamepieceZone getGamepieceZone() {
    return zone;
  }
}
