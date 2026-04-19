// Copyright (c) JSim contributors.
// Example robot simulation integration using JSim API

package frc.robot;

import jsim.PhysicsWorld;
import jsim.PhysicsBody;
import jsim.Vec3;
import jsim.nt.WorldPosePublisher;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private PhysicsWorld world;
    private PhysicsBody body;
    private WorldPosePublisher posePublisher;

    @Override
    public void simulationInit() {
        // Create a simulation world with 20ms timestep, gravity enabled
        world = new PhysicsWorld(0.02, true);

        // Create a body with 5kg mass
        body = world.createBody(5.0);

        // Set initial position
        body.setPosition(new Vec3(0.0, 0.0, 1.0));

        // Set initial velocity
        body.setLinearVelocity(new Vec3(0.0, 0.0, 0.0));

        // Set up publisher for up to 10 bodies
        posePublisher = new WorldPosePublisher(world.getHandle(), 10);
    }

    @Override
    public void simulationPeriodic() {
        // Example: move the body upward slowly
        body.setLinearVelocity(new Vec3(0.0, 0.0, 0.1));

        // Step the simulation
        world.step();

        // Publish the current world state to NetworkTables
        posePublisher.publishFrame();
    }

    @Override
    public void simulationExit() {
        // Clean up resources
        posePublisher.close();
        world.close();
    }
}
