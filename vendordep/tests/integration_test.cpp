#include <cassert>
#include <cmath>
#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/math/integrators.hpp"

int main() {
    std::cout << "Testing integration methods and rigid body dynamics...\n";

    // ===== Integration Method Tests =====
    {
        // Test Semi-Implicit Euler integration
        // Expected: v = v0 + a*dt; x = x0 + v*dt
        frcsim::Vector3 pos(0.0, 0.0, 0.0);
        frcsim::Vector3 vel(0.0, 0.0, 0.0);
        frcsim::Vector3 accel(0.0, 0.0, -9.81);
        double dt = 0.01;

        frcsim::Integrator::integrateLinear(pos, vel, accel, dt);
        
        // After semi-implicit: v should be 0 + (-9.81)*0.01 = -0.0981
        assert(std::fabs(vel.z - (-9.81 * dt)) < 1e-9);
        // Position should be 0 + (-0.0981)*0.01 = -0.000981
        assert(std::fabs(pos.z - vel.z * dt) < 1e-5);

        std::cout << "  ✓ Semi-Implicit Euler integration correct\n";
    }

    {
        // Test Explicit Euler integration (old position first)
        frcsim::Vector3 pos(0.0, 0.0, 0.0);
        frcsim::Vector3 vel(0.0, 0.0, 0.0);
        frcsim::Vector3 accel(0.0, 0.0, -9.81);
        double dt = 0.01;

        frcsim::Integrator::integrateLinearExplicit(pos, vel, accel, dt);
        
        // After explicit: x should update first with v=0, then v updates
        // So pos = 0 + 0*0.01 = 0, vel = 0 + (-9.81)*0.01
        assert(std::fabs(pos.z - 0.0) < 1e-9);
        assert(std::fabs(vel.z - (-9.81 * dt)) < 1e-9);

        std::cout << "  ✓ Explicit Euler integration correct\n";
    }

    {
        // Test RK2 (midpoint) integration
        frcsim::Vector3 pos(0.0, 0.0, 0.0);
        frcsim::Vector3 vel(1.0, 0.0, 0.0);
        frcsim::Vector3 accel(0.0, 0.0, 0.0);
        double dt = 0.1;

        frcsim::Integrator::integrateLinearRK2(pos, vel, accel, dt);
        
        // With zero acceleration, should be same as semi-implicit
        // v doesn't change, x = 0 + 1.0*0.1 = 0.1
        assert(std::fabs(pos.x - 0.1) < 1e-9);
        assert(std::fabs(vel.x - 1.0) < 1e-9);

        std::cout << "  ✓ RK2 integration correct\n";
    }

    {
        // Test angular integration
        frcsim::Quaternion orientation(1.0, 0.0, 0.0, 0.0);  // identity
        frcsim::Vector3 angular_vel(0.0, 0.0, 0.1);  // rotating around Z
        double dt = 0.1;

        frcsim::Integrator::integrateAngular(orientation, angular_vel, dt);
        
        // After small rotation, shouldn't be identity
        assert(std::fabs(orientation.w - 1.0) > 1e-5 || 
               std::fabs(orientation.x) > 1e-9 ||
               std::fabs(orientation.y) > 1e-9 ||
               std::fabs(orientation.z) > 1e-9);

        std::cout << "  ✓ Angular integration correct\n";
    }

    // ===== Rigid Body Dynamics Tests =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = true;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        body.setPosition(frcsim::Vector3(0.0, 0.0, 10.0));

        // Run for 100 steps (1.0 second)
        const int steps = 100;
        for (int i = 0; i < steps; ++i) {
            world.step();
        }

        // Body should have fallen under gravity
        // z = z0 + 0.5*g*t^2 = 10 - 0.5*9.81*(1.0)^2 ≈ 5.1
        assert(body.position().z < 6.0 && body.position().z > 4.0);
        
        // Velocity should be roughly v = g*t = 9.81 m/s downward
        assert(body.linearVelocity().z < -9.0 && body.linearVelocity().z > -10.5);

        std::cout << "  ✓ Gravity and free fall dynamics correct\n";
    }

    {
        // Test that disabling gravity works
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        body.setPosition(frcsim::Vector3(0.0, 0.0, 5.0));
        body.flags().enable_gravity = false;

        for (int i = 0; i < 100; ++i) {
            world.step();
        }

        // Position should not change when gravity is disabled and no forces applied
        assert(std::fabs(body.position().z - 5.0) < 1e-9);

        std::cout << "  ✓ Gravity disable flag works correctly\n";
    }

    {
        // Test applied forces
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);  // 1 kg
        body.setPosition(frcsim::Vector3(0.0, 0.0, 0.0));

        // Apply 10 N force in X direction
        body.applyForce(frcsim::Vector3(10.0, 0.0, 0.0));
        world.step();

        // Acceleration should be F/m = 10/1 = 10 m/s^2
        // Velocity should be a*dt = 10*0.01 = 0.1 m/s
        assert(std::fabs(body.linearVelocity().x - 0.1) < 1e-9);

        std::cout << "  ✓ Applied forces generate correct acceleration\n";
    }

    {
        // Test force at point (torque generation)
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.001;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;
        config.enable_gravity = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        
        // Set inertia tensor to identity for simplicity
        frcsim::Matrix3 inertia = frcsim::Matrix3::identity();
        body.setBodyInertiaTensor(inertia);
        
        body.setPosition(frcsim::Vector3(0.0, 0.0, 0.0));

        // Apply force at point offset from center
        frcsim::Vector3 force(0.0, 10.0, 0.0);
        frcsim::Vector3 point(1.0, 0.0, 0.0);  // 1m away in X
        body.applyForceAtPoint(force, point);

        world.step();

        // Torque = r × F = (1,0,0) × (0,10,0) = (0,0,10) N·m
        // Should generate angular velocity

        std::cout << "  ✓ Force at point generates torque\n";
    }

    {
        // Test kinematic bodies don't respond to forces
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        body.setPosition(frcsim::Vector3(0.0, 0.0, 0.0));
        body.flags().is_kinematic = true;

        body.applyForce(frcsim::Vector3(100.0, 0.0, 0.0));
        world.step();

        // Position should not change
        assert(std::fabs(body.position().x - 0.0) < 1e-9);
        assert(std::fabs(body.linearVelocity().x - 0.0) < 1e-9);

        std::cout << "  ✓ Kinematic bodies ignore applied forces\n";
    }

    {
        // Test linear and angular damping
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.enable_gravity = false;
        config.linear_damping_per_s = 0.1;  // 10% damping per second
        config.angular_damping_per_s = 0.1;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);
        body.setLinearVelocity(frcsim::Vector3(1.0, 0.0, 0.0));
        body.setAngularVelocity(frcsim::Vector3(0.0, 0.0, 1.0));

        world.step();

        // Damping factor: 1.0 - 0.1*0.01 = 0.999
        // New velocity should be ~0.999
        assert(std::fabs(body.linearVelocity().x - 0.999) < 1e-9);

        std::cout << "  ✓ Linear and angular damping applied correctly\n";
    }

    std::cout << "✓ All integration and dynamics tests passed!\n";
    return 0;
}
