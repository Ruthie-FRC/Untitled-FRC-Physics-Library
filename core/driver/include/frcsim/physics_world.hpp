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

/**
 * @brief Global runtime settings for PhysicsWorld dynamics and features.
 */
struct PhysicsConfig {
  /** @brief Fixed timestep duration in seconds used by step(). */
  double fixed_dt_s{0.01};
  /** @brief Integration scheme for position/rotation updates. */
  IntegrationMethod integration_method{IntegrationMethod::kSemiImplicitEuler};

  /** @brief Enable collision detection and response (currently unused). */
  bool enable_collision_detection{false};
  /** @brief Enable joint constraint solving during integration. */
  bool enable_joint_constraints{false};
  /** @brief Enable aerodynamic drag and Magnus lift computation. */
  bool enable_aerodynamics{false};
  /** @brief Enable per-body gravity acceleration. */
  bool enable_gravity{true};

  /** @brief Gravity acceleration vector in m/s^2 (typically {0, 0, -9.81}). */
  Vector3 gravity_mps2{0.0, 0.0, -9.81};
  /** @brief Linear velocity damping coefficient in 1/s (exponential decay). */
  double linear_damping_per_s{0.0};
  /** @brief Angular velocity damping coefficient in 1/s (exponential decay). */
  double angular_damping_per_s{0.0};
};

/**
 * @brief Unified physics scene manager for rigid bodies, assemblies, balls, and forces.
 *
 * The world integrates rigid body dynamics, joint constraints, and gameplay-specific
 * simulators (ball physics, turret mechanics) with configurable feature flags and
 * timestep behavior.
 */
class PhysicsWorld {
 public:
  /**
   * @brief Constructs a physics world with given configuration.
   * @param config Initial world settings and feature flags.
   */
  explicit PhysicsWorld(const PhysicsConfig& config = PhysicsConfig())
      : config_(config) {}

  /**
   * @brief Creates and registers a dynamic rigid body.
   * @param mass_kg Body mass in kilograms.
   * @return Reference to the new rigid body.
   */
  RigidBody& createBody(double mass_kg);
  /**
   * @brief Creates and registers an assembly of constrained rigid bodies.
   * @return Reference to the new rigid assembly.
   */
  RigidAssembly& createAssembly();

  /**
   * @brief Adds a collision boundary (plane or complex obstacle).
   * @return Reference to the new boundary.
   */
  EnvironmentalBoundary& addBoundary();
  /** @brief Mutable access to registered boundaries. */
  std::vector<EnvironmentalBoundary>& boundaries() { return boundaries_; }
  /** @brief Immutable access to registered boundaries. */
  const std::vector<EnvironmentalBoundary>& boundaries() const {
    return boundaries_;
  }

  /**
   * @brief Registers a global force generator (e.g., GravityForce, DragModel).
   * @param generator Shared pointer to the force generator.
   */
  void addGlobalForceGenerator(
      const std::shared_ptr<ForceGenerator>& generator);

  /** @brief Mutable access to all rigid bodies in the world. */
  std::vector<RigidBody>& bodies() { return bodies_; }
  /** @brief Immutable access to all rigid bodies in the world. */
  const std::vector<RigidBody>& bodies() const { return bodies_; }

  /** @brief Mutable access to all rigid assemblies in the world. */
  std::vector<RigidAssembly>& assemblies() { return assemblies_; }
  /** @brief Immutable access to all rigid assemblies in the world. */
  const std::vector<RigidAssembly>& assemblies() const { return assemblies_; }

  /**
   * @brief Creates and registers a ball physics simulator.
   * @param config Ball and environment configuration.
   * @param properties Ball physical properties (mass, radius, etc.).
   * @return Reference to the new ball simulator.
   */
  BallPhysicsSim3D& createBall(
      const BallPhysicsSim3D::Config& config = BallPhysicsSim3D::Config(),
      const BallPhysicsSim3D::BallProperties& properties =
          BallPhysicsSim3D::BallProperties());
  /** @brief Mutable access to all ball simulators. */
  std::vector<BallPhysicsSim3D>& balls() { return balls_; }
  /** @brief Immutable access to all ball simulators. */
  const std::vector<BallPhysicsSim3D>& balls() const { return balls_; }

  /**
   * @brief Advances all world entities by fixed_dt_s.
   *
   * Integrates rigid body dynamics, applies global forces, moves balls,
   * and updates constraints according to enabled feature flags.
   */
  void step();

  /** @brief Returns the number of step() calls executed so far. */
  std::size_t stepCount() const { return step_count_; }
  /** @brief Returns total simulated time in seconds. */
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
