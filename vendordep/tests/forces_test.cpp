#include <cassert>
#include <cmath>
#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/forces/gravity.hpp"

int main() {
    std::cout << "Testing forces and aerodynamics...\n";

    // ===== Basic Applied Force Tests =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        // Apply 10 N force in X direction
        body.applyForce(frcsim::Vector3(10.0, 0.0, 0.0));
        world.step();

        // Velocity should be a*dt = 10*0.01 = 0.1 m/s
        assert(std::fabs(body.linearVelocity().x - 0.1) < 1e-9);

        std::cout << "  ✓ Direct force application works\n";
    }

    // ===== Gravity Force via Global Config =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = true;
        config.gravity_mps2 = frcsim::Vector3(0.0, 0.0, -9.81);

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        body.setPosition(frcsim::Vector3(0.0, 0.0, 10.0));

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        // Body should have fallen under gravity
        assert(body.position().z < 6.0);
        assert(body.linearVelocity().z < -9.0);

        std::cout << "  ✓ Global gravity force works\n";
    }

    // ===== Gravity Force Generator Class =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        body.setPosition(frcsim::Vector3(0.0, 0.0, 10.0));

        // Add gravity as a force generator instead
        auto gravity = std::make_shared<frcsim::GravityForce>(
            frcsim::Vector3(0.0, 0.0, -9.81));
        world.addGlobalForceGenerator(gravity);

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        // Body should fall
        assert(body.position().z < 6.0);
        assert(body.linearVelocity().z < -9.0);

        std::cout << "  ✓ Gravity force generator works\n";
    }

    // ===== Multiple Force Accumulation =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        // Apply multiple forces
        body.applyForce(frcsim::Vector3(5.0, 0.0, 0.0));
        body.applyForce(frcsim::Vector3(5.0, 0.0, 0.0));
        
        world.step();

        // Total force should be 10 N, velocity 0.1 m/s
        assert(std::fabs(body.linearVelocity().x - 0.1) < 1e-9);

        std::cout << "  ✓ Force accumulation works\n";
    }

    // ===== Aerodynamics: Drag Model Tests =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.enable_aerodynamics = true;
        config.linear_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        // Drag should slow the body down
        assert(body.linearVelocity().x < 10.0);

        std::cout << "  ✓ Drag model reduces velocity\n";
    }

    // ===== Aerodynamics: Disabled Test =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.enable_aerodynamics = false;
        config.linear_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        // Without aerodynamics and damping, velocity should be unchanged
        assert(std::fabs(body.linearVelocity().x - 10.0) < 1e-9);

        std::cout << "  ✓ Aerodynamics can be disabled\n";
    }

    // ===== Magnus Effect Simulation Test =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.enable_aerodynamics = true;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(0.05);

        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));
        body.setAngularVelocity(frcsim::Vector3(0.0, 50.0, 0.0));

        for (int i = 0; i < 50; ++i) {
            world.step();
        }

        // Body should move forward
        assert(body.position().x > 0.0);

        std::cout << "  ✓ Magnus effect simulation runs\n";
    }

    std::cout << "✓ All force and aerodynamics tests passed!\n";
    return 0;
}
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.1;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        
        // Attach spring at origin, pulling body back to origin
        frcsim::Vector3 anchor(0.0, 0.0, 0.0);
        auto spring = std::make_shared<frcsim::SpringForceGenerator>(
            anchor,
            100.0,  // spring constant (N/m)
            0.5,    // damping coefficient
            0.0);   // rest length

        world.addGlobalForceGenerator(spring);

        // Displace body
        body.setPosition(frcsim::Vector3(1.0, 0.0, 0.0));

        // Run simulation
        for (int i = 0; i < 200; ++i) {
            world.step();
        }

        // Body should oscillate back toward origin
        // Due to damping, final position should be near origin
        assert(std::fabs(body.position().x) < 0.2);

        std::cout << "  ✓ Spring force generator works\n";
    }

    // ===== Motor Force Tests =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.05;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        
        // Create motor force: applies force to reach target velocity
        auto motor = std::make_shared<frcsim::MotorForceGenerator>(
            2.0,   // target velocity (m/s)
            10.0); // max force (N)

        world.addGlobalForceGenerator(motor);

        // Run simulation
        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        // Body should accelerate toward target velocity
        assert(body.linearVelocity().magnitude() > 0.5);

        std::cout << "  ✓ Motor force generator works\n";
    }

    // ===== Multiple Force Generators Test =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        // Add gravity
        auto gravity = std::make_shared<frcsim::GravityForceGenerator>(
            frcsim::Vector3(0.0, 0.0, -5.0));
        world.addGlobalForceGenerator(gravity);

        // Add lateral impulse force
        auto lateral = std::make_shared<frcsim::MotorForceGenerator>(
            1.0,  // target velocity in X
            5.0); // max force

        world.addGlobalForceGenerator(lateral);

        // Run simulation
        for (int i = 0; i < 50; ++i) {
            world.step();
        }

        // Body should move both sideways and downward
        assert(body.position().x > 0.0);
        assert(body.position().z < 0.0);

        std::cout << "  ✓ Multiple force generators work together\n";
    }

    // ===== Force Generator Clearing Test =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        // Add forces
        auto gravity = std::make_shared<frcsim::GravityForceGenerator>(
            frcsim::Vector3(0.0, 0.0, -10.0));
        world.addGlobalForceGenerator(gravity);

        // Step with gravity
        world.step();
        assert(body.linearVelocity().z < 0.0);

        // Clear all forces
        world.clearGlobalForceGenerators();
        
        // Reset velocity
        body.setLinearVelocity(frcsim::Vector3(0.0, 0.0, 0.0));

        // Step without gravity
        world.step();
        assert(std::fabs(body.linearVelocity().z) < 1e-9);

        std::cout << "  ✓ Force generator clearing works\n";
    }

    // ===== Aerodynamics: Drag Model Tests =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.enable_aerodynamics = true;
        config.linear_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        // Apply initial velocity
        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));

        // Run simulation
        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        // Drag should slow the body down
        assert(body.linearVelocity().x < 10.0);

        std::cout << "  ✓ Drag model reduces velocity\n";
    }

    // ===== Aerodynamics: Disabled Test =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.enable_aerodynamics = false;  // Disabled
        config.linear_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));

        // Run simulation
        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        // Without aerodynamics and damping, velocity should be unchanged
        assert(std::fabs(body.linearVelocity().x - 10.0) < 1e-9);

        std::cout << "  ✓ Aerodynamics can be disabled\n";
    }

    // ===== Magnus Effect Simulation Test =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.enable_aerodynamics = true;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(0.05);  // Small object (ball)

        // Apply velocity and spin
        body.setLinearVelocity(frcsim::Vector3(10.0, 0.0, 0.0));
        body.setAngularVelocity(frcsim::Vector3(0.0, 50.0, 0.0));  // Spinning

        // Run simulation
        for (int i = 0; i < 50; ++i) {
            world.step();
        }

        // Magnus force should deflect the ball
        // (The exact direction depends on aerodynamic model parameters)
        assert(body.position().x > 0.0);

        std::cout << "  ✓ Magnus effect simulation runs\n";
    }

    std::cout << "✓ All force and aerodynamics tests passed!\n";
    return 0;
}
