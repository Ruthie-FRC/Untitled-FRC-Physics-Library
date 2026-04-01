#include "frcsim/physics_world.hpp"

#include <cstdint>

namespace frcsim {

PhysicsWorld::PhysicsWorld(const PhysicsConfig& config)
: config_(config),
  drag_model_(0.47, 0.02),
  magnus_model_(1e-4),
  spin_decay_model_(1e-4) {}

PhysicsConfig& PhysicsWorld::config() { return config_; }

const PhysicsConfig& PhysicsWorld::config() const { return config_; }

RigidBody& PhysicsWorld::createBody(double mass_kg) {
bodies_.emplace_back(mass_kg);
return bodies_.back();
}

std::vector<RigidBody>& PhysicsWorld::bodies() { return bodies_; }

const std::vector<RigidBody>& PhysicsWorld::bodies() const { return bodies_; }

EnvironmentalBoundary& PhysicsWorld::addBoundary() {
boundaries_.emplace_back();
return boundaries_.back();
}

std::vector<EnvironmentalBoundary>& PhysicsWorld::boundaries() { return boundaries_; }

const std::vector<EnvironmentalBoundary>& PhysicsWorld::boundaries() const {
return boundaries_;
}

void PhysicsWorld::addGlobalForceGenerator(std::shared_ptr<ForceGenerator> generator) {
if (generator) {
global_force_generators_.push_back(std::move(generator));
}
}

void PhysicsWorld::clearGlobalForceGenerators() {
global_force_generators_.clear();
}

void PhysicsWorld::step(double dt_s) {
const double effective_dt_s = (dt_s > 0.0) ? dt_s : config_.fixed_dt_s;
if (effective_dt_s <= 0.0) return;

for (RigidBody& body : bodies_) {
body.clearAccumulators();
}

applyGlobalForces(effective_dt_s);

if (config_.enable_aerodynamics) {
applyAeroForces();
}

const Vector3 gravity = config_.enable_gravity ? config_.gravity_mps2 : Vector3::zero();
for (RigidBody& body : bodies_) {
body.integrate(
effective_dt_s,
config_.integration_method,
gravity,
config_.linear_damping_per_s,
config_.angular_damping_per_s);
}

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

double PhysicsWorld::accumulatedSimTimeS() const { return accumulated_sim_time_s_; }

std::uint64_t PhysicsWorld::stepCount() const { return step_count_; }

void PhysicsWorld::applyGlobalForces(double dt_s) {
for (RigidBody& body : bodies_) {
// Respect per-body enabled flags
if (body.flags().is_kinematic || body.isStatic()) continue;
for (const auto& generator : global_force_generators_) {
generator->apply(body, dt_s);
}
}
}

void PhysicsWorld::applyAeroForces() {
for (RigidBody& body : bodies_) {
if (body.isStatic() || body.flags().is_kinematic) continue;
drag_model_.apply(body);
magnus_model_.apply(body);
spin_decay_model_.apply(body);
}
}

void PhysicsWorld::solveCollisions(double /*dt_s*/) {
// TODO: Hook to collision_detector/contact_solver modules once broadphase/narrowphase are finalized.
// Respect body flags: enable_collisions, enable_friction.
// Use material properties: restitution, friction coefficients.
}

void PhysicsWorld::solveJointConstraints(double /*dt_s*/) {
// TODO: Hook to iterative joint solver after joint Jacobian API is finalized.
// Respect per-body flag: enable_kinematic_constraints.
}

void PhysicsWorld::applyBoundaryConstraints(double /*dt_s*/) {
// TODO: Enforce boundary constraints (walls, planes, boxes).
// For static constraints: clamp body position/velocity to boundary.
// For rigid-body boundaries: apply contact forces/impulses.
// Respect boundary state: is_active flag.
}

}  // namespace frcsim
