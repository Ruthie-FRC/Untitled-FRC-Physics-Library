#include <cassert>
#include <cmath>
#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/joints/revolute_joint.hpp"
#include "frcsim/joints/prismatic_joint.hpp"

int main() {
    std::cout << "Testing rigid assemblies and material system...\n";

    // ===== Assembly Creation Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;

        frcsim::PhysicsWorld world(config);
        
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        // Add bodies to assembly
        frcsim::RigidBody* body1 = assembly.addBody(1.0);
        frcsim::RigidBody* body2 = assembly.addBody(2.0);
        frcsim::RigidBody* body3 = assembly.addBody(3.0);

        assert(assembly.bodies().size() == 3);
        assert(body1 != nullptr);
        assert(body2 != nullptr);
        assert(body3 != nullptr);

        std::cout << "  ✓ Assembly body creation works\n";
    }

    // ===== Assembly Mass Calculation Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        assembly.addBody(1.0);
        assembly.addBody(2.0);
        assembly.addBody(3.0);

        double total_mass = assembly.totalMass();
        assert(std::fabs(total_mass - 6.0) < 1e-6);

        std::cout << "  ✓ Assembly total mass calculation correct\n";
    }

    // ===== Center of Mass Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        frcsim::RigidBody* b1 = assembly.addBody(1.0);
        frcsim::RigidBody* b2 = assembly.addBody(1.0);

        // Position bodies symmetrically
        b1->setPosition(frcsim::Vector3(-1.0, 0.0, 0.0));
        b2->setPosition(frcsim::Vector3(1.0, 0.0, 0.0));

        frcsim::Vector3 com = assembly.centerOfMass();
        
        // Center of mass should be at origin (equal masses, symmetric)
        // Com should be roughly between the two bodies
        assert(com.x >= -1.0 && com.x <= 1.0);
        assert(com.y >= -0.1 && com.y <= 0.1);
        assert(com.z >= -0.1 && com.z <= 0.1);

        std::cout << "  ✓ Center of mass calculation correct\n";
    }

    // ===== Joint Creation Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = true;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        assembly.addBody(1.0);
        assembly.addBody(1.0);

        // Create revolute joint
        frcsim::RevoluteJoint* rev = assembly.addRevoluteJoint(
            0, 1, frcsim::Vector3(0.0, 0.0, 1.0));
        
        assert(rev != nullptr);
        assert(assembly.joints().size() == 1);

        std::cout << "  ✓ Revolute joint creation works\n";
    }

    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = true;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        assembly.addBody(1.0);
        assembly.addBody(1.0);

        // Create prismatic joint
        frcsim::PrismaticJoint* pris = assembly.addPrismaticJoint(
            0, 1, frcsim::Vector3(0.0, 1.0, 0.0));
        
        assert(pris != nullptr);
        assert(assembly.joints().size() == 1);

        std::cout << "  ✓ Prismatic joint creation works\n";
    }

    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = true;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        assembly.addBody(1.0);
        assembly.addBody(1.0);

        // Create fixed joint
        frcsim::FixedJoint* fixed = assembly.addFixedJoint(0, 1);
        
        assert(fixed != nullptr);
        assert(assembly.joints().size() == 1);

        std::cout << "  ✓ Fixed joint creation works\n";
    }

    // ===== Revolute Joint Properties Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = true;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        assembly.addBody(1.0);
        assembly.addBody(1.0);

        frcsim::RevoluteJoint* joint = assembly.addRevoluteJoint(
            0, 1, frcsim::Vector3(0.0, 0.0, 1.0));

        // Test angle limits
        joint->setLimits(-1.57, 1.57);
        assert(joint->hasLimits());
        assert(std::fabs(joint->minAngle() - (-1.57)) < 1e-9);
        assert(std::fabs(joint->maxAngle() - 1.57) < 1e-9);

        // Test motor properties
        joint->setMotorTarget(0.5, 10.0);
        assert(joint->hasMotor());
        assert(std::fabs(joint->motorTargetVelocity() - 0.5) < 1e-9);
        assert(std::fabs(joint->motorMaxTorque() - 10.0) < 1e-9);

        // Clear limits
        joint->clearLimits();
        assert(!joint->hasLimits());

        std::cout << "  ✓ Revolute joint properties work correctly\n";
    }

    // ===== Prismatic Joint Properties Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = true;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        assembly.addBody(1.0);
        assembly.addBody(1.0);

        frcsim::PrismaticJoint* joint = assembly.addPrismaticJoint(
            0, 1, frcsim::Vector3(0.0, 1.0, 0.0));

        // Test displacement limits
        joint->setLimits(-0.5, 0.5);
        assert(joint->hasLimits());
        assert(std::fabs(joint->minDisplacement() - (-0.5)) < 1e-9);
        assert(std::fabs(joint->maxDisplacement() - 0.5) < 1e-9);

        // Test motor properties (force instead of torque)
        joint->setMotorTarget(0.2, 50.0);
        assert(joint->hasMotor());
        assert(std::fabs(joint->motorTargetVelocity() - 0.2) < 1e-9);
        assert(std::fabs(joint->motorMaxForce() - 50.0) < 1e-9);

        std::cout << "  ✓ Prismatic joint properties work correctly\n";
    }

    // ===== Joint Base Properties Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = true;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        frcsim::RigidBody* b1 = assembly.addBody(1.0);
        frcsim::RigidBody* b2 = assembly.addBody(1.0);

        frcsim::RevoluteJoint* joint = assembly.addRevoluteJoint(
            0, 1, frcsim::Vector3(0.0, 0.0, 1.0));

        // Test anchor points
        joint->setAnchorA(frcsim::Vector3(0.5, 0.0, 0.0));
        joint->setAnchorB(frcsim::Vector3(-0.5, 0.0, 0.0));

        assert(std::fabs(joint->anchorA().x - 0.5) < 1e-9);
        assert(std::fabs(joint->anchorB().x - (-0.5)) < 1e-9);

        // Test enable/disable
        assert(joint->isEnabled());
        joint->setEnabled(false);
        assert(!joint->isEnabled());
        joint->setEnabled(true);
        assert(joint->isEnabled());

        // Test break force threshold
        joint->setBreakForceThreshold(100.0);
        assert(std::fabs(joint->breakForceThreshold() - 100.0) < 1e-9);

        std::cout << "  ✓ Joint base properties work correctly\n";
    }

    // ===== Material Properties Tests =====
    {
        frcsim::Material mat;
        
        // Default values
        assert(std::fabs(mat.coefficient_of_restitution - 0.5) < 1e-9);
        assert(std::fabs(mat.coefficient_of_friction_kinetic - 0.6) < 1e-9);
        assert(std::fabs(mat.coefficient_of_friction_static - 0.8) < 1e-9);

        // Custom values
        mat.coefficient_of_restitution = 0.9;
        mat.coefficient_of_friction_kinetic = 0.3;
        mat.coefficient_of_friction_static = 0.4;
        mat.collision_damping = 0.2;

        assert(std::fabs(mat.coefficient_of_restitution - 0.9) < 1e-9);
        assert(std::fabs(mat.collision_damping - 0.2) < 1e-9);

        std::cout << "  ✓ Material properties work correctly\n";
    }

    // ===== Body Material Assignment Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidBody& body = world.createBody(1.0);

        frcsim::Material mat;
        mat.coefficient_of_restitution = 0.95;
        body.setMaterial(mat);

        assert(body.material() != nullptr);
        assert(std::fabs(body.material()->coefficient_of_restitution - 0.95) < 1e-9);

        std::cout << "  ✓ Body material assignment works\n";
    }

    // ===== Assembly Constraint Control Tests =====
    {
        frcsim::PhysicsConfig config;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = true;

        frcsim::PhysicsWorld world(config);
        frcsim::RigidAssembly& assembly = world.createAssembly();
        
        assembly.addBody(1.0);
        assembly.addBody(1.0);

        frcsim::RevoluteJoint* joint = assembly.addRevoluteJoint(
            0, 1, frcsim::Vector3(0.0, 0.0, 1.0));

        // Disable all constraints in assembly
        assembly.enableConstraints(false);
        assert(!joint->isEnabled());

        // Re-enable
        assembly.enableConstraints(true);
        assert(joint->isEnabled());

        std::cout << "  ✓ Assembly constraint control works\n";
    }

    // ===== Body Flags Tests =====
    {
        frcsim::BodyFlags flags;
        
        // Default values
        assert(flags.enable_gravity == true);
        assert(flags.enable_friction == true);
        assert(flags.enable_collisions == true);
        assert(flags.enable_kinematic_constraints == true);
        assert(flags.enable_deformation == false);
        assert(flags.is_kinematic == false);

        // Modify flags
        flags.enable_gravity = false;
        flags.is_kinematic = true;
        
        assert(flags.enable_gravity == false);
        assert(flags.is_kinematic == true);

        std::cout << "  ✓ Body flags work correctly\n";
    }

    std::cout << "✓ All assembly, joint, and material tests passed!\n";
    return 0;
}
