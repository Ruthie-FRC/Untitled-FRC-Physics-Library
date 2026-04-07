#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "frcsim/aerodynamics/drag_model.hpp"
#include "frcsim/aerodynamics/magnus_model.hpp"
#include "frcsim/aerodynamics/spin_decay_model.hpp"
#include "frcsim/config/physics_config.hpp"
#include "frcsim/field/boundary.hpp"
#include "frcsim/forces/force_generator.hpp"
#include "frcsim/rigidbody/rigid_assembly.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

class PhysicsWorld {
  public:
explicit PhysicsWorld(const PhysicsConfig& config = PhysicsConfig());

PhysicsConfig& config();
const PhysicsConfig& config() const;

// Individual body management
RigidBody& createBody(double mass_kg = 1.0);
std::vector<RigidBody>& bodies();
const std::vector<RigidBody>& bodies() const;

// Assembly (hierarchical) management
RigidAssembly& createAssembly();
std::vector<RigidAssembly>& assemblies();
const std::vector<RigidAssembly>& assemblies() const;

// Environmental boundaries
EnvironmentalBoundary& addBoundary();
std::vector<EnvironmentalBoundary>& boundaries();
const std::vector<EnvironmentalBoundary>& boundaries() const;

// Force generators
void addGlobalForceGenerator(std::shared_ptr<ForceGenerator> generator);
void clearGlobalForceGenerators();

// Main simulation step
void step(double dt_s = -1.0);

double accumulatedSimTimeS() const;
std::uint64_t stepCount() const;

// Get all bodies (individual + assembly bodies) for iteration
void forEachBody(const std::function<void(RigidBody&)>& callback);
void forEachBody(const std::function<void(const RigidBody&)>& callback) const;

  private:
void applyGlobalForces(double dt_s);
void applyAeroForces();
void solveCollisions(double dt_s);
void solveJointConstraints(double dt_s);
void applyBoundaryConstraints(double dt_s);

PhysicsConfig config_;
std::vector<RigidBody> bodies_;
std::vector<RigidAssembly> assemblies_;
std::vector<EnvironmentalBoundary> boundaries_;
std::vector<std::shared_ptr<ForceGenerator>> global_force_generators_;

DragModel drag_model_;
MagnusModel magnus_model_;
SpinDecayModel spin_decay_model_;

double accumulated_sim_time_s_{0.0};
std::uint64_t step_count_{0};
};

}  // namespace frcsim
