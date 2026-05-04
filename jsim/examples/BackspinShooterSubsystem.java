package examples;

import api.GamepieceZone;
import api.SimRobot;
import api.Rotation3d;

/**
 * A subsystem representing a shooter that imparts backspin on the game piece.
 *
 * <p>This class uses two {@link GamepieceZone} instances to simulate a main flywheel and a
 * secondary roller that applies backspin. The main flywheel provides the primary exit velocity,
 * while the top roller's speed relative to the main flywheel creates the spin.
 */
public class BackspinShooterSubsystem {

  private final GamepieceZone mainFlywheelZone;
  private final GamepieceZone backspinRollerZone;

  private double mainFlywheelVelocity = 10.0; // m/s
  private double backspinRollerVelocity = 15.0; // m/s, faster to create spin
  private Rotation3d exitRotation = new Rotation3d(0, 0, 0);

  /**
   * Constructs a new BackspinShooterSubsystem.
   *
   * @param robot The {@link SimRobot} this subsystem is a part of.
   */
  public BackspinShooterSubsystem(SimRobot robot) {
    // The main flywheel provides the primary force.
    this.mainFlywheelZone = new GamepieceZone(robot);

    // The backspin roller is another zone, typically positioned above the main one.
    // Its differential speed creates the spin.
    this.backspinRollerZone = new GamepieceZone(robot);

    // Start with both zones disabled.
    this.mainFlywheelZone.setMode(GamepieceZone.Mode.DISABLED);
    this.backspinRollerZone.setMode(GamepieceZone.Mode.DISABLED);
  }

  /**
   * Configures the parameters for the next shot.
   *
   * @param mainVelocity The velocity of the main flywheel (m/s).
   * @param backspinVelocity The velocity of the top backspin roller (m/s). To create backspin,
   *     this should typically be greater than the main flywheel velocity.
   * @param rotation The exit angle of the game piece.
   */
  public void setShot(double mainVelocity, double backspinVelocity, Rotation3d rotation) {
    this.mainFlywheelVelocity = mainVelocity;
    this.backspinRollerVelocity = backspinVelocity;
    this.exitRotation = rotation;
  }

  /**
   * Activates the shooter to launch a game piece.
   *
   * <p>This enables both the main flywheel and the backspin roller zones. The combination of the
   * two will launch the game piece with both linear and angular velocity.
   */
  public void shoot() {
    mainFlywheelZone.setExitParameters(mainFlywheelVelocity, exitRotation);
    mainFlywheelZone.setMode(GamepieceZone.Mode.SHOOT);

    backspinRollerZone.setExitParameters(backspinRollerVelocity, exitRotation);
    backspinRollerZone.setMode(GamepieceZone.Mode.SHOOT);
  }

  /** Deactivates the shooter. */
  public void stop() {
    mainFlywheelZone.setMode(GamepieceZone.Mode.DISABLED);
    backspinRollerZone.setMode(GamepieceZone.Mode.DISABLED);
  }
}
