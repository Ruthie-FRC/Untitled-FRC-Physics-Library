package examples;

import api.SimRobot;
import api.Rotation3d;

/**
 * Example RobotContainer for wiring up FRC-style subsystems and commands.
 * This class demonstrates how to instantiate and connect the FlywheelSubsystem and FlywheelHoodSubsystem
 * for both real and simulated robot code.
 */
public class RobotContainer {
    private final SimRobot robot;
    private final FlywheelSubsystem flywheelSubsystem;
    private final FlywheelHoodSubsystem flywheelHoodSubsystem;

    public RobotContainer(SimRobot robot) {
        this.robot = robot;
        this.flywheelSubsystem = new FlywheelSubsystem(robot);
        this.flywheelHoodSubsystem = new FlywheelHoodSubsystem(robot);
    }

    public void runFlywheelExample() {
        // Example: Set flywheel to 15 m/s left, 15 m/s right, shoot at 30 degrees up
        flywheelSubsystem.setFlywheel(15.0, 15.0, new Rotation3d(0, Math.toRadians(-30), 0));
        flywheelSubsystem.shoot();
        // ...simulate time passing, then stop
        flywheelSubsystem.stop();
    }

    public void runHoodedShooterExample() {
        // Example: Set flywheel to 18 m/s, backspin roller to 22 m/s, shoot at 45 degrees up
        flywheelHoodSubsystem.setShot(18.0, 18.0, 22.0, new Rotation3d(0, Math.toRadians(-45), 0));
        flywheelHoodSubsystem.shoot();
        // ...simulate time passing, then stop
        flywheelHoodSubsystem.stop();
    }

    public FlywheelSubsystem getFlywheelSubsystem() {
        return flywheelSubsystem;
    }

    public FlywheelHoodSubsystem getFlywheelHoodSubsystem() {
        return flywheelHoodSubsystem;
    }
}
