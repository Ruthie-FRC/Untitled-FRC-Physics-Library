// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include <iostream>

#include "frcsim/physics_world.hpp"

int main() {
  frcsim::PhysicsConfig config;
  config.fixed_dt_s = 0.01;
  config.integration_method = frcsim::IntegrationMethod::kSemiImplicitEuler;
  config.enable_collision_detection = false;
  config.enable_joint_constraints = false;

  frcsim::PhysicsWorld world(config);
  frcsim::RigidBody& body = world.createBody(2.0);
  body.setPosition(frcsim::Vector3(0.0, 0.0, 1.0));
  body.setLinearVelocity(frcsim::Vector3(2.0, 0.0, 0.0));

  for (int i = 0; i < 200; ++i) {
    world.step();
  }

  const frcsim::Vector3& p = body.position();
  const frcsim::Vector3& v = body.linearVelocity();

  std::cout << "t=" << world.accumulatedSimTimeS() << "s\n";
  std::cout << "position=" << p << "\n";
  std::cout << "velocity=" << v << "\n";
  return 0;
}
