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

        body.applyForce(frcsim::Vector3(10.0, 0.0, 0.0));
        world.step();

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

        auto gravity = std::make_shared<frcsim::GravityForce>(
            frcsim::Vector3(0.0, 0.0, -9.81));
        world.addGlobalForceGenerator(gravity);

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

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

        body.applyForce(frcsim::Vector3(5.0, 0.0, 0.0));
        body.applyForce(frcsim::Vector3(5.0, 0.0, 0.0));
        
        world.step();

        assert(std::fabs(body.linearVelocity().x - 0.1) < 1e-6);
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

        assert(std::fabs(body.linearVelocity().x - 10.0) < 1e-6);
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

        assert(body.position().x > 0.0);
        std::cout << "  ✓ Magnus effect simulation runs\n";
    }

    std::cout << "✓ All force and aerodynamics tests passed!\n";
    return 0;
}
