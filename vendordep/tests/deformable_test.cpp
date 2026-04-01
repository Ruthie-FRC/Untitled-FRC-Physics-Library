#include <cassert>
#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/rigidbody/deformable_body.hpp"

int main() {
    std::cout << "Testing deformable bodies...\n";

    // ===== Deformable Body Creation Tests =====
    {
        frcsim::DeformableBody def_body(2.0);
        
        // Check rigid base properties
        assert(std::fabs(def_body.rigidBase().massKg() - 2.0) < 1e-6);
        assert(!def_body.isDeformationEnabled());

        std::cout << "  ✓ Deformable body creation works\n";
    }

    // ===== Deformable Body Stiffness Tests =====
    {
        frcsim::DeformableBody def_body(2.0);
        
        def_body.setBendStiffness(150.0);
        assert(std::fabs(def_body.bendStiffness() - 150.0) < 1e-6);

        std::cout << "  ✓ Bend stiffness settable\n";
    }

    // ===== Deformable Body Damping Tests =====
    {
        frcsim::DeformableBody def_body(2.0);
        
        def_body.setWarpDamping(25.0);
        assert(std::fabs(def_body.warpDamping() - 25.0) < 1e-6);

        std::cout << "  ✓ Warp damping settable\n";
    }

    // ===== Deformable Body Enable/Disable Tests =====
    {
        frcsim::DeformableBody def_body(2.0);
        
        assert(!def_body.isDeformationEnabled());
        
        def_body.enableDeformation(true);
        assert(def_body.isDeformationEnabled());
        
        def_body.enableDeformation(false);
        assert(!def_body.isDeformationEnabled());

        std::cout << "  ✓ Deformation enable/disable works\n";
    }

    // ===== Deformation Nodes Management Tests =====
    {
        frcsim::DeformableBody def_body(2.0);
        
        // Initially empty
        assert(def_body.deformationNodes().size() == 0);
        
        // Add some nodes
        def_body.deformationNodes().push_back(frcsim::Vector3(0.0, 0.0, 0.0));
        def_body.deformationNodes().push_back(frcsim::Vector3(0.1, 0.0, 0.0));
        def_body.deformationNodes().push_back(frcsim::Vector3(0.2, 0.0, 0.0));
        
        assert(def_body.deformationNodes().size() == 3);

        std::cout << "  ✓ Deformation nodes manageable\n";
    }

    // ===== Deformation Velocities Management Tests =====
    {
        frcsim::DeformableBody def_body(2.0);
        
        // Initially empty
        assert(def_body.deformationVelocities().size() == 0);
        
        // Add velocities
        def_body.deformationVelocities().push_back(frcsim::Vector3(0.0, 0.0, 0.0));
        def_body.deformationVelocities().push_back(frcsim::Vector3(0.05, 0.0, 0.0));
        
        assert(def_body.deformationVelocities().size() == 2);

        std::cout << "  ✓ Deformation velocities manageable\n";
    }

    // ===== Rigid Base Access Tests =====
    {
        frcsim::DeformableBody def_body(3.0);
        
        def_body.rigidBase().setPosition(frcsim::Vector3(1.0, 2.0, 3.0));
        def_body.rigidBase().setLinearVelocity(frcsim::Vector3(0.5, 0.0, 0.0));
        
        assert(std::fabs(def_body.rigidBase().position().x - 1.0) < 1e-6);
        assert(std::fabs(def_body.rigidBase().linearVelocity().x - 0.5) < 1e-6);

        std::cout << "  ✓ Rigid base properties accessible\n";
    }

    // ===== Deformable Body Flags Tests =====
    {
        frcsim::DeformableBody def_body(2.0);
        
        // Access body flags through rigid base
        def_body.rigidBase().flags().enable_deformation = true;
        assert(def_body.rigidBase().flags().enable_deformation);
        
        def_body.rigidBase().flags().enable_deformation = false;
        assert(!def_body.rigidBase().flags().enable_deformation);

        std::cout << "  ✓ Deformable body flags work through rigid base\n";
    }

    // ===== Deformable Body in Physics World Tests =====
    {
        frcsim::PhysicsConfig config;
        config.fixed_dt_s = 0.01;
        config.enable_collision_detection = false;
        config.enable_joint_constraints = false;
        config.linear_damping_per_s = 0.0;
        config.angular_damping_per_s = 0.0;

        frcsim::PhysicsWorld world(config);
        
        frcsim::RigidBody& body = world.createBody(2.0);
        
        // Enable deformation through flags
        body.flags().enable_deformation = true;
        
        body.setPosition(frcsim::Vector3(0.0, 0.0, 5.0));

        // Run simulation
        for (int i = 0; i < 50; ++i) {
            world.step();
        }

        // Body should simulate normally; deformation mesh not yet implemented
        // so it acts as a rigid body with flag set
        assert(body.position().z < 5.0);

        std::cout << "  ✓ Deformable body integrates with physics world\n";
    }

    // ===== Deformable Body Configuration Scenario =====
    {
        // Create a deformable object representing a spring-like material
        frcsim::DeformableBody spring(0.5);
        
        spring.enableDeformation(true);
        spring.setBendStiffness(200.0);
        spring.setWarpDamping(15.0);
        
        // Set up deformation mesh (example: linked nodes)
        spring.deformationNodes().push_back(frcsim::Vector3(0.0, 0.0, 0.0));
        spring.deformationNodes().push_back(frcsim::Vector3(0.05, 0.0, 0.0));
        spring.deformationNodes().push_back(frcsim::Vector3(0.10, 0.0, 0.0));
        
        spring.deformationVelocities().resize(3);
        
        assert(spring.isDeformationEnabled());
        assert(spring.deformationNodes().size() == 3);
        assert(std::fabs(spring.bendStiffness() - 200.0) < 1e-6);

        std::cout << "  ✓ Complex deformable body configuration works\n";
    }

    std::cout << "✓ All deformable body tests passed!\n";
    return 0;
}
