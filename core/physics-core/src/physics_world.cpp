// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "frcsim/physics_world.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace frcsim {

PhysicsWorld::PhysicsWorld(const PhysicsConfig& config)
    : config_(config),
      drag_model_(0.47, 0.02),
      magnus_model_(1e-4),
      spin_decay_model_(1e-4) {}

PhysicsConfig& PhysicsWorld::config() {
  return config_;
}

const PhysicsConfig& PhysicsWorld::config() const {
  return config_;
}

RigidBody& PhysicsWorld::createBody(double mass_kg) {
  bodies_.emplace_back(mass_kg);
  return bodies_.back();
}

std::vector<RigidBody>& PhysicsWorld::bodies() {
  return bodies_;
}

const std::vector<RigidBody>& PhysicsWorld::bodies() const {
  return bodies_;
}

RigidAssembly& PhysicsWorld::createAssembly() {
  assemblies_.emplace_back();
  return assemblies_.back();
}

std::vector<RigidAssembly>& PhysicsWorld::assemblies() {
  return assemblies_;
}

const std::vector<RigidAssembly>& PhysicsWorld::assemblies() const {
  return assemblies_;
}

EnvironmentalBoundary& PhysicsWorld::addBoundary() {
  boundaries_.emplace_back();
  return boundaries_.back();
}

std::vector<EnvironmentalBoundary>& PhysicsWorld::boundaries() {
  return boundaries_;
}

const std::vector<EnvironmentalBoundary>& PhysicsWorld::boundaries() const {
  return boundaries_;
}

void PhysicsWorld::addGlobalForceGenerator(
    std::shared_ptr<ForceGenerator> generator) {
  if (generator) {
    global_force_generators_.push_back(std::move(generator));
  }
}

void PhysicsWorld::clearGlobalForceGenerators() {
  global_force_generators_.clear();
}

void PhysicsWorld::forEachBody(
    const std::function<void(RigidBody&)>& callback) {
  for (RigidBody& body : bodies_) {
    callback(body);
  }
  for (RigidAssembly& assembly : assemblies_) {
    for (RigidBody& body : assembly.bodies()) {
      callback(body);
    }
  }
}

void PhysicsWorld::forEachBody(
    const std::function<void(const RigidBody&)>& callback) const {
  for (const RigidBody& body : bodies_) {
    callback(body);
  }
  for (const RigidAssembly& assembly : assemblies_) {
    for (const RigidBody& body : assembly.bodies()) {
      callback(body);
    }
  }
}

void PhysicsWorld::step(double dt_s) {
  const double effective_dt_s = (dt_s > 0.0) ? dt_s : config_.fixed_dt_s;
  if (effective_dt_s <= 0.0)
    return;

  forEachBody([](RigidBody& body) { body.clearAccumulators(); });

  applyGlobalForces(effective_dt_s);

  if (config_.enable_aerodynamics) {
    applyAeroForces();
  }

  const Vector3 gravity =
      config_.enable_gravity ? config_.gravity_mps2 : Vector3::zero();
  forEachBody([&](RigidBody& body) {
    body.integrate(effective_dt_s, config_.integration_method, gravity,
                   config_.linear_damping_per_s, config_.angular_damping_per_s);
  });

  if (config_.enable_collision_detection) {
    solveCollisions(effective_dt_s);
  }

  if (config_.enable_joint_constraints) {
    solveJointConstraints(effective_dt_s);
  }

  applyBoundaryConstraints(effective_dt_s);

  accumulated_sim_time_s_ += effective_dt_s;
  ++step_count_;
}

double PhysicsWorld::accumulatedSimTimeS() const {
  return accumulated_sim_time_s_;
}

std::uint64_t PhysicsWorld::stepCount() const {
  return step_count_;
}

void PhysicsWorld::applyGlobalForces(double dt_s) {
  forEachBody([&](RigidBody& body) {
    if (body.flags().is_kinematic || body.isStatic())
      return;
    for (const auto& generator : global_force_generators_) {
      generator->apply(body, dt_s);
    }
  });
}

void PhysicsWorld::applyAeroForces() {
  forEachBody([this](RigidBody& body) {
    if (body.isStatic() || body.flags().is_kinematic)
      return;
    const auto drag = drag_model_.computeForceDetailed(body);
    body.applyForce(Vector3(drag.force.x, drag.force.y, drag.force.z));
    magnus_model_.apply(body);
    spin_decay_model_.apply(body);
  });
}

void PhysicsWorld::solveCollisions(double /*dt_s*/) {
  // TODO: Hook to collision_detector/contact_solver modules once
  // broadphase/narrowphase are finalized. Respect body flags:
  // enable_collisions, enable_friction. Use material properties: restitution,
  // friction coefficients.
}

void PhysicsWorld::solveJointConstraints(double dt_s) {
  // Solve constraints for all assemblies
  for (RigidAssembly& assembly : assemblies_) {
    if (!assembly.bodies().empty()) {
      assembly.solveConstraints(dt_s, config_.solver_iterations);
    }
  }
  // TODO: Hook for future global joint system or inter-body constraints.
}

void PhysicsWorld::applyBoundaryConstraints(double /*dt_s*/) {
  // TODO: Enforce boundary constraints (walls, planes, boxes).
  // For static constraints: clamp body position/velocity to boundary.
  // For rigid-body boundaries: apply contact forces/impulses.
  // Respect boundary state: is_active flag.
}

}  // namespace frcsim
