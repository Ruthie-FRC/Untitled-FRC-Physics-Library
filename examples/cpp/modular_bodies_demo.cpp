// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/rigidbody/deformable_body.hpp"
#include "frcsim/rigidbody/material.hpp"

int main() {
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = 0.01;
  config.integration_method = frcsim::IntegrationMethod::kSemiImplicitEuler;
  config.enable_collision_detection = false;
  config.enable_joint_constraints = false;
  config.linear_damping_per_s = 0.05;

  frcsim::PhysicsWorld world(config);

  // Example 1: Normal rigid body with gravity enabled
  frcsim::RigidBody& box1 = world.createBody(2.0);
  box1.setPosition(frcsim::Vector3(0.0, 0.0, 5.0));
  box1.flags().enable_gravity = true;
  box1.flags().enable_friction = true;

  // Set material with high restitution (bouncy)
  frcsim::Material bouncy_mat;
  bouncy_mat.coefficient_of_restitution = 0.8;
  bouncy_mat.coefficient_of_friction_kinetic = 0.3;
  box1.setMaterial(bouncy_mat);

  std::cout << "Box 1 (with gravity, restitution="
            << box1.material()->coefficient_of_restitution << ")\n";

  // Example 2: Kinematic body (controlled externally, ignores physics forces)
  frcsim::RigidBody& platform = world.createBody(10.0);
  platform.setPosition(frcsim::Vector3(0.0, 0.0, 0.0));
  platform.flags().is_kinematic = true;
  platform.flags().enable_collisions = true;

  std::cout << "Platform (kinematic, position locked by external control)\n";

  // Example 3: Body with gravity disabled (floats)
  frcsim::RigidBody& balloon = world.createBody(0.1);
  balloon.setPosition(frcsim::Vector3(2.0, 0.0, 2.0));
  balloon.flags().enable_gravity = false;
  balloon.setLinearVelocity(frcsim::Vector3(1.0, 0.0, 0.0));

  std::cout << "Balloon (gravity disabled, will maintain altitude)\n";

  // Example 4: Environmental boundary (field wall)
  frcsim::EnvironmentalBoundary& wall = world.addBoundary();
  wall.type = frcsim::BoundaryType::kPlane;
  wall.behavior = frcsim::BoundaryBehavior::kStaticConstraint;
  wall.position_m = frcsim::Vector3(5.0, 0.0, 0.0);
  wall.is_active = true;

  std::cout << "Boundary added (wall at x=5.0m)\n";

  // Run simulation for 200 steps
  for (int i = 0; i < 200; ++i) {
    world.step();
  }

  const frcsim::Vector3& p1 = box1.position();
  const frcsim::Vector3& p_balloon = balloon.position();

  std::cout << "\nAfter " << world.stepCount() << " steps ("
            << world.accumulatedSimTimeS() << "s):\n";
  std::cout << "Box1 position: " << p1 << "\n";
  std::cout << "Balloon position: " << p_balloon << "\n";

  // Example 5: Deformable body (bendable, with warping toggle)
  frcsim::DeformableBody def_body(1.5);
  def_body.enableDeformation(true);
  def_body.setBendStiffness(50.0);
  def_body.setWarpDamping(5.0);
  def_body.rigidBase().setPosition(frcsim::Vector3(0.0, 5.0, 3.0));

  std::cout << "\nDeformable body created:\n";
  std::cout << "  Deformation enabled: "
            << (def_body.isDeformationEnabled() ? "true" : "false") << "\n";
  std::cout << "  Bend stiffness: " << def_body.bendStiffness() << " N/m\n";
  std::cout << "  Warp damping: " << def_body.warpDamping() << " N·s/m\n";

  return 0;
}
