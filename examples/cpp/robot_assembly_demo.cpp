// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include <iostream>

#include "frcsim/physics_world.hpp"
#include "frcsim/joints/prismatic_joint.hpp"
#include "frcsim/joints/revolute_joint.hpp"

int main() {
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = 0.01;
  config.integration_method = frcsim::IntegrationMethod::kSemiImplicitEuler;
  config.enable_collision_detection = false;
  config.enable_joint_constraints = true;
  config.linear_damping_per_s = 0.05;

  frcsim::PhysicsWorld world(config);

  // Create a simple robot assembly: base + arm with revolute joint + gripper
  // with prismatic joint
  frcsim::RigidAssembly& robot = world.createAssembly();

  // Add bodies: 0 = chassis, 1 = arm, 2 = gripper
  frcsim::RigidBody* chassis = robot.addBody(5.0);
  frcsim::RigidBody* arm = robot.addBody(2.0);
  frcsim::RigidBody* gripper = robot.addBody(0.5);

  // Set initial positions in local assembly frame
  chassis->setPosition(frcsim::Vector3(0.0, 0.0, 0.5));
  arm->setPosition(frcsim::Vector3(0.0, 0.5, 0.5));
  gripper->setPosition(frcsim::Vector3(0.0, 1.0, 0.5));

  // Create revolute joint between chassis and arm (hinge at shoulder)
  frcsim::RevoluteJoint* shoulder = robot.addRevoluteJoint(
      0, 1, frcsim::Vector3(0.0, 0.0, 1.0));  // Z-axis rotation
  if (shoulder) {
    shoulder->setAnchorA(frcsim::Vector3(0.0, 0.5, 0.0));    // Local to chassis
    shoulder->setAnchorB(frcsim::Vector3(0.0, -0.25, 0.0));  // Local to arm
    shoulder->setLimits(-1.57, 1.57);                        // +/- 90 degrees
    shoulder->setMotorTarget(0.5,
                             10.0);  // Target 0.5 rad/s with 10 N·m max torque
  }

  // Create prismatic joint between arm and gripper (linear extension)
  frcsim::PrismaticJoint* extension = robot.addPrismaticJoint(
      1, 2, frcsim::Vector3(0.0, 1.0, 0.0));  // Y-axis translation
  if (extension) {
    extension->setAnchorA(frcsim::Vector3(0.0, 0.5, 0.0));  // Local to arm
    extension->setAnchorB(
        frcsim::Vector3(0.0, -0.25, 0.0));  // Local to gripper
    extension->setLimits(-0.1, 0.3);        // Can extend 0.3m, retract 0.1m
    extension->setMotorTarget(0.2, 5.0);    // Target 0.2 m/s with 5 N max force
  }

  std::cout << "Robot assembly created:\n";
  std::cout << "  Chassis mass: " << chassis->massKg() << " kg\n";
  std::cout << "  Arm mass: " << arm->massKg() << " kg\n";
  std::cout << "  Gripper mass: " << gripper->massKg() << " kg\n";
  std::cout << "  Total assembly mass: " << robot.totalMass() << " kg\n";
  std::cout << "  Joints: " << robot.joints().size() << "\n";

  // Run simulation for 300 steps (3 seconds)
  for (int i = 0; i < 300; ++i) {
    world.step();
  }

  std::cout << "\nAfter " << world.stepCount() << " steps ("
            << world.accumulatedSimTimeS() << "s):\n";
  std::cout << "Chassis position: " << chassis->position() << "\n";
  std::cout << "Arm position: " << arm->position() << "\n";
  std::cout << "Gripper position: " << gripper->position() << "\n";
  std::cout << "Assembly CoM: " << robot.centerOfMass() << "\n";

  if (shoulder) {
    std::cout << "\nShoulder joint:\n";
    std::cout << "  Enabled: " << (shoulder->isEnabled() ? "true" : "false")
              << "\n";
    std::cout << "  Motor torque: " << shoulder->motorMaxTorque() << " N·m\n";
    std::cout << "  Target velocity: " << shoulder->motorTargetVelocity()
              << " rad/s\n";
    std::cout << "  Has angle limits: "
              << (shoulder->hasLimits() ? "true" : "false") << "\n";
  }

  if (extension) {
    std::cout << "\nExtension joint:\n";
    std::cout << "  Enabled: " << (extension->isEnabled() ? "true" : "false")
              << "\n";
    std::cout << "  Motor force: " << extension->motorMaxForce() << " N\n";
    std::cout << "  Target velocity: " << extension->motorTargetVelocity()
              << " m/s\n";
    std::cout << "  Has displacement limits: "
              << (extension->hasLimits() ? "true" : "false") << "\n";
  }

  return 0;
}
