// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

/**
 * @file robot_assembly_demo.cpp
 * @brief Demonstrates creating a multi-body robot with joints.
 *
 * This example shows how to build a complex mechanism by connecting multiple
 * rigid bodies together using joints. The concepts shown are:
 * 1. Creating a `RigidAssembly` to group related bodies and joints.
 * 2. Adding multiple bodies (chassis, arm, gripper) to the assembly.
 * 3. Connecting bodies with a `RevoluteJoint` (a hinge).
 * 4. Connecting bodies with a `PrismaticJoint` (a slider).
 * 5. Configuring joint limits and motors.
 */

#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/joints/prismatic_joint.hpp"
#include "frcsim/joints/revolute_joint.hpp"

int main() {
  // --- World Configuration ---
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = 0.01; // 100 Hz simulation rate
  config.integration_method = frcsim::IntegrationMethod::kSemiImplicitEuler;
  config.enable_collision_detection = false; // Collisions are off for simplicity
  config.enable_joint_constraints = true; // Joints must be enabled!
  config.linear_damping_per_s = 0.05;

  frcsim::PhysicsWorld world(config);
  std::cout << "--- Robot Assembly Demo ---" << std::endl;

  // 1. Create a RigidAssembly.
  // An assembly is a container for a group of bodies and joints that are
  // connected, like a robot arm or a whole robot.
  frcsim::RigidAssembly& robot = world.createAssembly();
  std::cout << "\nCreated a new RigidAssembly for the robot." << std::endl;

  // 2. Add bodies to the assembly.
  // We'll create a simple robot with a chassis, an arm, and a gripper.
  // The integer returned is the body's index within the assembly.
  frcsim::RigidBody* chassis = robot.addBody(5.0); // 5.0 kg
  frcsim::RigidBody* arm = robot.addBody(2.0);     // 2.0 kg
  frcsim::RigidBody* gripper = robot.addBody(0.5); // 0.5 kg

  // Set initial positions relative to the assembly's origin.
  chassis->setPosition(frcsim::Vector3(0.0, 0.0, 0.5));
  arm->setPosition(frcsim::Vector3(0.0, 0.5, 0.5));
  gripper->setPosition(frcsim::Vector3(0.0, 1.0, 0.5));

  std::cout << "Added 3 bodies: chassis, arm, and gripper." << std::endl;

  // 3. Create a Revolute Joint (Hinge).
  // This will connect the chassis (body 0) to the arm (body 1).
  // The joint will rotate around the Z-axis.
  frcsim::RevoluteJoint* shoulder = robot.addRevoluteJoint(
      0, 1, frcsim::Vector3(0.0, 0.0, 1.0)); // Z-axis rotation
  if (shoulder) {
    // Anchors define the pivot point in the local coordinates of each body.
    shoulder->setAnchorA(frcsim::Vector3(0.0, 0.25, 0.0)); // Pivot on chassis
    shoulder->setAnchorB(frcsim::Vector3(0.0, -0.25, 0.0)); // Pivot on arm
    // Set angular limits for the joint.
    shoulder->setLimits(-1.57, 1.57); // +/- 90 degrees (in radians)
    // Set a motor to drive the joint towards a target velocity.
    shoulder->setMotorTarget(0.5, 10.0); // Target 0.5 rad/s with 10 N·m max torque
    std::cout << "Added a revolute 'shoulder' joint between chassis and arm." << std::endl;
  }

  // 4. Create a Prismatic Joint (Slider).
  // This will connect the arm (body 1) to the gripper (body 2).
  // The joint will translate along the arm's Y-axis.
  frcsim::PrismaticJoint* extension = robot.addPrismaticJoint(
      1, 2, frcsim::Vector3(0.0, 1.0, 0.0)); // Y-axis translation
  if (extension) {
    // Anchors define the connection point in the local coordinates of each body.
    extension->setAnchorA(frcsim::Vector3(0.0, 0.5, 0.0));  // Connection point on arm
    extension->setAnchorB(frcsim::Vector3(0.0, 0.0, 0.0));  // Connection point on gripper
    // Set linear limits for the joint.
    extension->setLimits(0.0, 0.3); // Can extend 0.3m from its starting point
    // Set a motor to drive the joint.
    extension->setMotorTarget(0.1, 5.0); // Target 0.1 m/s with 5 N max force
    std::cout << "Added a prismatic 'extension' joint between arm and gripper." << std::endl;
  }

  std::cout << "\n--- Initial State ---" << std::endl;
  std::cout << "Total assembly mass: " << robot.totalMass() << " kg\n";
  std::cout << "Number of joints: " << robot.joints().size() << "\n";

  // --- Simulation Loop ---
  const int num_steps = 300; // Simulate for 3 seconds
  std::cout << "\nRunning simulation for " << num_steps << " steps..." << std::endl;
  for (int i = 0; i < num_steps; ++i) {
    world.step();
  }

  // --- Final State ---
  std::cout << "\n--- Simulation Finished ---" << std::endl;
  std::cout << "Total simulated time: " << world.accumulatedSimTimeS() << "s\n";
  std::cout << "Final Chassis position: " << chassis->position() << "\n";
  std::cout << "Final Arm position: " << arm->position() << "\n";
  std::cout << "Final Gripper position: " << gripper->position() << "\n";
  std::cout << "Final Assembly Center of Mass: " << robot.centerOfMass() << "\n";

  if (shoulder) {
    std::cout << "\nShoulder Joint Final State:\n";
    std::cout << "  Angle: " << shoulder->getAngle() << " rad\n";
  }

  if (extension) {
    std::cout << "\nExtension Joint Final State:\n";
    std::cout << "  Displacement: " << extension->getDisplacement() << " m\n";
  }

  return 0;
}
