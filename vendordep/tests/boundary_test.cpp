#include <cassert>
#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/field/boundary.hpp"

int main() {
    std::cout << "Testing environmental boundaries and field objects...\n";

    // ===== Boundary Creation Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);
        
        frcsim::EnvironmentalBoundary& boundary = world.addBoundary();
        
        assert(world.boundaries().size() == 1);
        assert(boundary.is_active);
        
        std::cout << "  ✓ Boundary creation works\n";
    }

    // ===== Boundary Type Configuration Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);
        
        {
            frcsim::EnvironmentalBoundary& wall = world.addBoundary();
            wall.type = frcsim::BoundaryType::kWall;
            assert(wall.type == frcsim::BoundaryType::kWall);
        }

        {
            frcsim::EnvironmentalBoundary& plane = world.addBoundary();
            plane.type = frcsim::BoundaryType::kPlane;
            assert(plane.type == frcsim::BoundaryType::kPlane);
        }

        {
            frcsim::EnvironmentalBoundary& box = world.addBoundary();
            box.type = frcsim::BoundaryType::kBox;
            assert(box.type == frcsim::BoundaryType::kBox);
        }

        {
            frcsim::EnvironmentalBoundary& cylinder = world.addBoundary();
            cylinder.type = frcsim::BoundaryType::kCylinder;
            assert(cylinder.type == frcsim::BoundaryType::kCylinder);
        }

        assert(world.boundaries().size() == 4);

        std::cout << "  ✓ All boundary types can be created\n";
    }

    // ===== Boundary Behavior Configuration Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);
        
        {
            frcsim::EnvironmentalBoundary& b1 = world.addBoundary();
            b1.behavior = frcsim::BoundaryBehavior::kRigidBody;
            assert(b1.behavior == frcsim::BoundaryBehavior::kRigidBody);
        }

        {
            frcsim::EnvironmentalBoundary& b2 = world.addBoundary();
            b2.behavior = frcsim::BoundaryBehavior::kStaticConstraint;
            assert(b2.behavior == frcsim::BoundaryBehavior::kStaticConstraint);
        }

        std::cout << "  ✓ Boundary behaviors configurable\n";
    }

    // ===== Boundary Position and Orientation Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);
        frcsim::EnvironmentalBoundary& boundary = world.addBoundary();

        // Set position
        boundary.position_m = frcsim::Vector3(1.0, 2.0, 3.0);
        assert(std::fabs(boundary.position_m.x - 1.0) < 1e-6);
        assert(std::fabs(boundary.position_m.y - 2.0) < 1e-6);
        assert(std::fabs(boundary.position_m.z - 3.0) < 1e-6);

        // Set orientation
        boundary.orientation = frcsim::Quaternion(1.0, 0.0, 0.0, 0.0);
        assert(std::fabs(boundary.orientation.w - 1.0) < 1e-6);

        std::cout << "  ✓ Boundary position and orientation configurable\n";
    }

    // ===== Boundary Geometry Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);
        
        {
            frcsim::EnvironmentalBoundary& box = world.addBoundary();
            box.type = frcsim::BoundaryType::kBox;
            box.half_extents_m = frcsim::Vector3(1.0, 2.0, 3.0);
            
            assert(std::fabs(box.half_extents_m.x - 1.0) < 1e-6);
            assert(std::fabs(box.half_extents_m.y - 2.0) < 1e-6);
            assert(std::fabs(box.half_extents_m.z - 3.0) < 1e-6);
        }

        {
            frcsim::EnvironmentalBoundary& cylinder = world.addBoundary();
            cylinder.type = frcsim::BoundaryType::kCylinder;
            cylinder.radius_m = 0.5;
            
            assert(std::fabs(cylinder.radius_m - 0.5) < 1e-6);
        }

        std::cout << "  ✓ Boundary geometry configurable\n";
    }

    // ===== Boundary Physical Properties Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);
        frcsim::EnvironmentalBoundary& boundary = world.addBoundary();

        boundary.restitution = 0.8;
        boundary.friction_coefficient = 0.6;

        assert(std::fabs(boundary.restitution - 0.8) < 1e-6);
        assert(std::fabs(boundary.friction_coefficient - 0.6) < 1e-6);

        std::cout << "  ✓ Boundary physical properties configurable\n";
    }

    // ===== Boundary Metadata Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);
        frcsim::EnvironmentalBoundary& boundary = world.addBoundary();

        // User tagging
        boundary.user_id = 42;
        assert(boundary.user_id == 42);

        // Active flag
        boundary.is_active = true;
        assert(boundary.is_active);
        
        boundary.is_active = false;
        assert(!boundary.is_active);

        std::cout << "  ✓ Boundary metadata works\n";
    }

    // ===== Multiple Boundaries Test =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);

        // Add several boundaries (typical FRC field layout)
        auto& north_wall = world.addBoundary();
        north_wall.type = frcsim::BoundaryType::kWall;
        north_wall.position_m = frcsim::Vector3(0.0, 8.23, 0.0);
        north_wall.user_id = 1;

        auto& south_wall = world.addBoundary();
        south_wall.type = frcsim::BoundaryType::kWall;
        south_wall.position_m = frcsim::Vector3(0.0, -8.23, 0.0);
        south_wall.user_id = 2;

        auto& goal_zone = world.addBoundary();
        goal_zone.type = frcsim::BoundaryType::kBox;
        goal_zone.position_m = frcsim::Vector3(4.0, 4.0, 0.5);
        goal_zone.user_id = 3;

        assert(world.boundaries().size() == 3);
        assert(world.boundaries()[0].user_id == 1);
        assert(world.boundaries()[1].user_id == 2);
        assert(world.boundaries()[2].user_id == 3);

        std::cout << "  ✓ Multiple boundaries managed correctly\n";
    }

    // ===== Boundary Normal Vector Test =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);
        frcsim::EnvironmentalBoundary& boundary = world.addBoundary();

        // Default normal should be Z-up
        const frcsim::Vector3& normal = boundary.normal();
        assert(std::fabs(normal.x - 0.0) < 1e-6);
        assert(std::fabs(normal.y - 0.0) < 1e-6);
        assert(std::fabs(normal.z - 1.0) < 1e-6);

        std::cout << "  ✓ Boundary normal vector correct\n";
    }

    // ===== Field Boundary Integration Test =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& robot = world.createBody(10.0);
        robot.setPosition(frcsim::Vector3(0.0, 0.0, 0.5));

        // Add field walls
        auto& north = world.addBoundary();
        north.type = frcsim::BoundaryType::kWall;
        north.position_m = frcsim::Vector3(0.0, 8.23, 0.0);
        north.behavior = frcsim::BoundaryBehavior::kStaticConstraint;

        auto& south = world.addBoundary();
        south.type = frcsim::BoundaryType::kWall;
        south.position_m = frcsim::Vector3(0.0, -8.23, 0.0);
        south.behavior = frcsim::BoundaryBehavior::kStaticConstraint;

        // Run simulation (collision solver not yet implemented, so no collision response)
        for (int i = 0; i < 10; ++i) {
            world.step();
        }

        // Verify boundaries are present
        assert(world.boundaries().size() == 2);

        std::cout << "  ✓ Field boundaries integrate with world\n";
    }

    std::cout << "✓ All boundary and field tests passed!\n";
    return 0;
}
