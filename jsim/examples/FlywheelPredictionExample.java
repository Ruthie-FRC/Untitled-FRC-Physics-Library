package examples;

import api.Gamepiece;
import api.Gamepiece.GamepieceType;
import api.SimRobot;
import api.Rotation3d;
import api.Transform3d;
import api.Translation2d;
import api.Translation3d;

/**
 * An example demonstrating how to use the {@link ShooterSubsystem} to launch a game piece and
 * observe its trajectory.
 */
public final class FlywheelPredictionExample {
  private FlywheelPredictionExample() {
    // Private constructor to prevent instantiation of this utility class.
  }

  public static void main(String[] args) throws InterruptedException {
    System.out.println("Starting Flywheel Prediction Example...");

    // 1. Create a simulated robot.
    // The Translation2d array defines the robot's chassis footprint.
    // For this example, we are not using a StateManager or a specific RobotID.
    SimRobot robot =
        SimRobot.createRobot(
            new Translation2d[] {
              new Translation2d(-0.5, -0.5),
              new Translation2d(0.5, -0.5),
              new Translation2d(0.5, 0.5),
              new Translation2d(-0.5, 0.5)
            },
            null,
            null);

    // Position the robot on the field.
    robot.setTransform(
        new Transform3d(new Translation3d(2.0, 4.0, 0.5), new Rotation3d(0, 0, 0)));
    System.out.println("✓ Robot created and placed on the field at " + robot.getTransform());

    // 2. Create the shooter subsystem.
    ShooterSubsystem shooter = new ShooterSubsystem(robot);
    System.out.println("✓ Shooter subsystem initialized.");

    // 3. Load a game piece into the robot.
    // We must first give the robot a game piece to shoot.
    Gamepiece ball = new Gamepiece(GamepieceType.BALL, 0.2); // 0.2 kg mass
    robot.loadGamepiece(ball);
    System.out.println("✓ Game piece loaded into the robot.");

    // 4. Configure the shot.
    double exitVelocity = 15.0; // meters per second
    // Shoot at a 45-degree angle upwards.
    Rotation3d exitAngle = new Rotation3d(0, Math.toRadians(-45), 0);
    shooter.setShot(exitVelocity, exitAngle);
    System.out.println(
        "✓ Shot configured: " + exitVelocity + " m/s at " + exitAngle.getAngle() + " radians.");

    // 5. Shoot the game piece.
    shooter.shoot();
    System.out.println("✓ Shooting!");

    // 6. Simulate and observe.
    // We will step the simulation and print the game piece's position.
    System.out.println("\n--- Simulation Log ---");
    for (int i = 0; i < 200; i++) {
      // In a real scenario, you would step the world here.
      // For this example, we'll assume a world step is happening.
      // Let's manually update the game piece's state for demonstration.
      // In a real simulation, the physics engine would do this.

      // After the first step, the shooter is done.
      if (i == 1) {
        shooter.stop();
      }

      // Check if the game piece has been released yet.
      if (robot.getGamepieceCount() == 0) {
        // Once shot, the game piece is no longer associated with the robot.
        // In a full simulation, you would get the game piece from the world.
        // Here, we'll just print a message every few steps.
        if (i % 20 == 0) {
          System.out.printf(
              "Step %03d: Game piece is in flight (position would be tracked by world state).%n",
              i);
        }
      } else {
        System.out.printf("Step %03d: Robot is holding the game piece.%n", i);
      }

      // Simulate a delay for real-time observation.
      Thread.sleep(50);
    }

    System.out.println("\n--- Simulation Finished ---");
    System.out.println(
        "Example complete. In a full JSim environment, you would see the ball's trajectory.");
  }
}
