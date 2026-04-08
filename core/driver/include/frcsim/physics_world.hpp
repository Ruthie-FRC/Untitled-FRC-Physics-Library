// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "frcsim/field/boundary.hpp"
#include "frcsim/forces/force_generator.hpp"
#include "frcsim/gamepiece/ball_physics.hpp"
#include "frcsim/rigidbody/rigid_assembly.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

struct PhysicsConfig {
  double fixed_dt_s{0.01};
  IntegrationMethod integration_method{IntegrationMethod::kSemiImplicitEuler};

  bool enable_collision_detection{false};
  bool enable_joint_constraints{false};
  bool enable_aerodynamics{false};
  bool enable_gravity{true};

  Vector3 gravity_mps2{0.0, 0.0, -9.81};
  double linear_damping_per_s{0.0};
  double angular_damping_per_s{0.0};
};

class PhysicsWorld {
 public:
  explicit PhysicsWorld(const PhysicsConfig& config = PhysicsConfig())
      : config_(config) {}

  RigidBody& createBody(double mass_kg);
  RigidAssembly& createAssembly();

  EnvironmentalBoundary& addBoundary();
  std::vector<EnvironmentalBoundary>& boundaries() { return boundaries_; }
  const std::vector<EnvironmentalBoundary>& boundaries() const {
    return boundaries_;
  }

  void addGlobalForceGenerator(
      const std::shared_ptr<ForceGenerator>& generator);

  std::vector<RigidBody>& bodies() { return bodies_; }
  const std::vector<RigidBody>& bodies() const { return bodies_; }

  std::vector<RigidAssembly>& assemblies() { return assemblies_; }
  const std::vector<RigidAssembly>& assemblies() const { return assemblies_; }

  BallPhysicsSim3D& createBall(
      const BallPhysicsSim3D::Config& config = BallPhysicsSim3D::Config(),
      const BallPhysicsSim3D::BallProperties& properties =
          BallPhysicsSim3D::BallProperties());
  std::vector<BallPhysicsSim3D>& balls() { return balls_; }
  const std::vector<BallPhysicsSim3D>& balls() const { return balls_; }

  void step();

  std::size_t stepCount() const { return step_count_; }
  double accumulatedSimTimeS() const { return accumulated_sim_time_s_; }

  PhysicsConfig& config() { return config_; }
  const PhysicsConfig& config() const { return config_; }

 private:
  PhysicsConfig config_{};

  std::vector<RigidBody> bodies_{};
  std::vector<RigidAssembly> assemblies_{};
  std::vector<BallPhysicsSim3D> balls_{};
  std::vector<EnvironmentalBoundary> boundaries_{};
  std::vector<std::shared_ptr<ForceGenerator>> global_force_generators_{};

  std::size_t step_count_{0};
  double accumulated_sim_time_s_{0.0};
};

}  // namespace frcsim
