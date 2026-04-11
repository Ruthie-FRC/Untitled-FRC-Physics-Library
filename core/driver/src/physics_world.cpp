// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "frcsim/physics_world.hpp"

#include <memory>

namespace frcsim {

RigidBody& PhysicsWorld::createBody(double mass_kg) {
  bodies_.emplace_back(mass_kg);
  if (!config_.enable_gravity) {
    bodies_.back().flags().enable_gravity = false;
  }
  return bodies_.back();
}

RigidAssembly& PhysicsWorld::createAssembly() {
  assemblies_.emplace_back();
  return assemblies_.back();
}

BallPhysicsSim3D& PhysicsWorld::createBall(
    const BallPhysicsSim3D::Config& config,
    const BallPhysicsSim3D::BallProperties& properties) {
  balls_.emplace_back(config, properties);
  return balls_.back();
}

EnvironmentalBoundary& PhysicsWorld::addBoundary() {
  boundaries_.emplace_back();
  return boundaries_.back();
}

void PhysicsWorld::addGlobalForceGenerator(
    const std::shared_ptr<ForceGenerator>& generator) {
  if (generator) {
    global_force_generators_.push_back(generator);
  }
}

void PhysicsWorld::step() {
  const double dt_s = config_.fixed_dt_s;

  auto step_body = [&](RigidBody& body) {
    if (config_.enable_aerodynamics && !body.flags().is_kinematic) {
      // TODO(placeholder-aero): Replace hard-coded drag params with
      // body/material/config-driven coefficients.
      const auto drag = Vector3::dragForceDetailed(
          body.linearVelocity(), 0.47,
          body.dragReferenceAreaM2(body.linearVelocity()) > 0.0
              ? body.dragReferenceAreaM2(body.linearVelocity())
              : 0.01);
      body.applyForce(Vector3(drag.force.x, drag.force.y, drag.force.z));

      // TODO(placeholder-aero): Route through aerodynamic models and expose
      // magnus tuning in PhysicsConfig.
      const Vector3 magnus = Vector3::magnusForce(body.linearVelocity(),
                                                  body.angularVelocity(), 1e-4);
      body.applyForce(magnus);
    }

    for (const auto& generator : global_force_generators_) {
      generator->apply(body, dt_s);
    }

    body.integrate(dt_s, config_.integration_method, config_.gravity_mps2,
                   config_.linear_damping_per_s, config_.angular_damping_per_s);
  };

  for (auto& body : bodies_) {
    step_body(body);
  }

  for (auto& assembly : assemblies_) {
    for (auto& body : assembly.bodies()) {
      step_body(body);
    }
    if (config_.enable_joint_constraints) {
      assembly.solveConstraints(dt_s, 4);
    }
  }

  for (auto& ball : balls_) {
    ball.step(dt_s);
  }

  ++step_count_;
  accumulated_sim_time_s_ += dt_s;
}

}  // namespace frcsim
