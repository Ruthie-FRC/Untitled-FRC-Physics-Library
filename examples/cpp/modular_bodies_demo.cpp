// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file modular_bodies_demo.cpp
 * @brief Demonstrates various types of rigid bodies and environmental objects.
 *
 * This example showcases different configurations and types of bodies in JSim:
 * 1. A standard dynamic body affected by gravity.
 * 2. A kinematic body, which is moved by code, not physics forces.
 * 3. A dynamic body with gravity disabled, causing it to float.
 * 4. An environmental boundary (a static wall).
 * 5. A deformable body that can bend and warp.
 */

#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/rigidbody/deformable_body.hpp"
#include "frcsim/rigidbody/material.hpp"

int main() {
  // --- World Configuration ---
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = 0.01; // 100 Hz simulation rate
  config.integration_method = frcsim::IntegrationMethod::kSemiImplicitEuler;
  // Enable collision detection to allow interactions between objects.
  config.enable_collision_detection = true;
  config.enable_joint_constraints = false;
  // Apply a small amount of linear damping to everything to prevent
  // things from moving forever.
  config.linear_damping_per_s = 0.05;

  frcsim::PhysicsWorld world(config);
  std::cout << "--- Modular Bodies Demo ---" << std::endl;

  // --- Example 1: Standard Dynamic Body ---
  // This is a normal physics object that falls under gravity and collides.
  frcsim::RigidBody& box1 = world.createBody(2.0); // 2.0 kg mass
  box1.setPosition(frcsim::Vector3(0.0, 0.0, 5.0)); // Start 5m high
  box1.flags().enable_gravity = true;
  box1.flags().enable_friction = true;

  // Set a custom material to make the box bouncy.
  frcsim::Material bouncy_mat;
  bouncy_mat.coefficient_of_restitution = 0.8; // 80% bouncy
  bouncy_mat.coefficient_of_friction_kinetic = 0.3;
  box1.setMaterial(bouncy_mat);

  std::cout << "\n1. Created a dynamic 'bouncy box' with gravity enabled." << std::endl;

  // --- Example 2: Kinematic Body ---
  // A kinematic body is not affected by forces (like gravity or collisions).
  // Instead, its position and velocity are set directly by user code.
  // It can still push other dynamic objects around. Useful for moving platforms.
  frcsim::RigidBody& platform = world.createBody(10.0);
  platform.setPosition(frcsim::Vector3(0.0, 0.0, 0.0));
  platform.flags().is_kinematic = true; // Set the kinematic flag
  platform.flags().enable_collisions = true;

  std::cout << "2. Created a 'kinematic platform' at ground level." << std::endl;

  // --- Example 3: Floating Body (Gravity Disabled) ---
  // This body will not be pulled down by gravity.
  frcsim::RigidBody& balloon = world.createBody(0.1); // 0.1 kg mass
  balloon.setPosition(frcsim::Vector3(2.0, 0.0, 2.0));
  balloon.flags().enable_gravity = false; // The key flag for floating
  balloon.setLinearVelocity(frcsim::Vector3(1.0, 0.0, 0.0)); // It will drift sideways

  std::cout << "3. Created a 'floating balloon' with gravity disabled." << std::endl;

  // --- Example 4: Environmental Boundary ---
  // Boundaries are static, immovable planes used for things like field walls.
  frcsim::EnvironmentalBoundary& wall = world.addBoundary();
  wall.type = frcsim::BoundaryType::kPlane;
  wall.behavior = frcsim::BoundaryBehavior::kStaticConstraint;
  // This plane is defined by its normal pointing in the -X direction,
  // located at X = 5.0. Nothing can pass it.
  wall.position_m = frcsim::Vector3(5.0, 0.0, 0.0);
  wall.is_active = true;

  std::cout << "4. Added a static boundary 'wall' at x=5.0m." << std::endl;

  // --- Simulation Loop ---
  const int num_steps = 300; // Simulate for 3 seconds (300 steps * 0.01s/step)
  std::cout << "\nRunning simulation for " << num_steps << " steps..." << std::endl;
  for (int i = 0; i < num_steps; ++i) {
    // In a real application, you might update the kinematic platform's
    // position here, e.g., platform.setPosition(...)
    world.step();
  }

  // --- Final State ---
  const frcsim::Vector3& p1 = box1.position();
  const frcsim::Vector3& p_balloon = balloon.position();

  std::cout << "\n--- Simulation Finished ---" << std::endl;
  std::cout << "Total simulated time: " << world.accumulatedSimTimeS() << "s\n";
  std::cout << "Bouncy box final position: " << p1 << " (should have bounced on the platform)\n";
  std::cout << "Floating balloon final position: " << p_balloon << " (should have drifted into the wall)\n";

  // --- Example 5: Deformable Body ---
  // A deformable body is a special type that can bend and warp.
  // It is not added to the main physics world in this example but is shown
  // for API demonstration purposes.
  frcsim::DeformableBody def_body(1.5); // 1.5 kg mass
  def_body.enableDeformation(true);
  def_body.setBendStiffness(50.0); // Resistance to bending
  def_body.setWarpDamping(5.0);    // How quickly it returns to its original shape
  def_body.rigidBase().setPosition(frcsim::Vector3(0.0, 5.0, 3.0));

  std::cout << "\n5. Deformable body API demo:\n";
  std::cout << "  Deformation enabled: "
            << (def_body.isDeformationEnabled() ? "true" : "false") << "\n";
  std::cout << "  Bend stiffness: " << def_body.bendStiffness() << " N/m\n";
  std::cout << "  Warp damping: " << def_body.warpDamping() << " N·s/m\n";

  return 0;
}
