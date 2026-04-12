// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "frcsim/field/boundary.hpp"
#include "frcsim/forces/force_generator.hpp"
#include "frcsim/gamepiece/ball_physics.hpp"
#include "frcsim/rigidbody/rigid_assembly.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

/**
 * @brief Global runtime settings for @ref PhysicsWorld dynamics and optional features.
 *
 * This structure centralizes tunable simulation constants so callers can choose
 * between physically richer behavior and lighter-weight execution profiles.
 *
 * Design notes:
 * - All values use SI units.
 * - Values are consumed every world step; updating them affects subsequent steps.
 * - Defaults favor stable behavior for FRC-style simulations.
 */
struct PhysicsConfig {
  /**
   * @brief Fixed timestep in seconds used by @ref PhysicsWorld::step.
   *
   * Typical values:
   * - 0.02 for 50 Hz simulation loops.
   * - 0.01 for higher fidelity at moderate compute cost.
   * - 0.005 for tighter contact/aero resolution at higher cost.
   */
  double fixed_dt_s{0.01};

  /**
   * @brief Integration scheme used for rigid body translational/rotational updates.
   *
   * See @ref IntegrationMethod for tradeoffs between speed and numerical behavior.
   */
  IntegrationMethod integration_method{IntegrationMethod::kSemiImplicitEuler};

  /**
   * @brief Enables broad collision/contact processing.
   *
   * This flag is provided for feature gating and future expansion. Some contact
   * paths may still execute depending on world internals and active boundaries.
   */
  bool enable_collision_detection{false};

  /**
   * @brief Enables joint/constraint solving for rigid assemblies.
   *
   * Disable for maximum throughput when assemblies are not used.
   */
  bool enable_joint_constraints{false};

  /**
   * @brief Enables aerodynamic drag and Magnus lift forces.
   *
   * When true, per-body aerodynamic geometry/material metadata is used when
   * available; otherwise world defaults are applied.
   */
  bool enable_aerodynamics{false};

  /**
   * @brief Enables gravity contribution during body integration.
   *
   * Individual bodies can additionally opt in/out via body flags.
   */
  bool enable_gravity{true};

  /**
   * @brief Default dimensionless drag coefficient (Cd).
   *
   * Used when a body does not provide an explicit drag coefficient.
   */
  double default_drag_coefficient{0.47};

  /**
   * @brief Default drag reference area in square meters.
   *
   * Used when no aerodynamic geometry is provided for a body.
   */
  double default_drag_reference_area_m2{0.01};

  /** @brief Ambient air density in kilograms per cubic meter. */
  double air_density_kgpm3{1.225};

  /**
   * @brief Linear drag coefficient in N/(m/s) for optional viscous damping.
   *
   * This term is applied in addition to quadratic drag when enabled.
   */
  double linear_drag_coefficient_n_per_mps{0.0};

  /**
   * @brief Magnus lift scaling coefficient.
   *
   * Magnus force is proportional to this value and typically computed from
   * angular and linear velocity cross terms.
   */
  double magnus_coefficient{1e-4};

  /**
   * @brief Gravity acceleration vector in meters per second squared.
   *
   * Common Earth-like default is {0, 0, -9.81}.
   */
  Vector3 gravity_mps2{0.0, 0.0, -9.81};

  /**
   * @brief Linear damping coefficient in 1/s.
   *
   * Applied as a per-step multiplicative attenuation to translational velocity.
   */
  double linear_damping_per_s{0.0};

  /**
   * @brief Angular damping coefficient in 1/s.
   *
   * Applied as a per-step multiplicative attenuation to angular velocity.
   */
  double angular_damping_per_s{0.0};
};

/**
 * @brief Unified physics scene manager for rigid bodies, assemblies, ball simulators, and global forces.
 *
 * @ref PhysicsWorld is the central orchestration object for the native simulation
 * runtime. It owns all registered entities and advances them in lock-step using
 * a fixed timestep from @ref PhysicsConfig.
 *
 * High-level responsibilities:
 * - Manage entity lifetimes (bodies, assemblies, ball simulators, boundaries).
 * - Apply global and per-body force models.
 * - Resolve boundary contacts with configurable restitution and friction.
 * - Apply optional aerodynamic drag and Magnus lift.
 * - Enforce broad interaction filtering via layer/mask bitsets.
 * - Provide material-pair interaction overrides for contact coefficients.
 *
 * Determinism and ordering:
 * - Simulation order is stable with respect to insertion order of entities.
 * - Calling code should avoid mutating world topology while stepping.
 * - Consistent inputs and step counts produce consistent outputs on the same build.
 */
class PhysicsWorld {
 public:
  /**
   * @brief Material-pair contact interaction override entry.
   *
   * This table row overrides default contact coefficients when a body with
   * `material_a_id` collides with a boundary or body using `material_b_id`.
   * Lookup is symmetric, so (A, B) and (B, A) are equivalent.
   */
  struct MaterialInteraction {
    /** @brief First material numeric id (application-defined namespace). */
    std::int32_t material_a_id{0};
    /** @brief Second material numeric id (application-defined namespace). */
    std::int32_t material_b_id{0};
    /** @brief Pair-specific restitution coefficient, typically in [0, 1]. */
    double restitution{0.5};
    /** @brief Pair-specific friction coefficient, typically >= 0. */
    double friction{0.6};
    /** @brief True when this row should be considered during lookup. */
    bool enabled{true};
  };

  /**
   * @brief Constructs a world with an initial configuration snapshot.
   * @param config Initial world settings copied into internal storage.
   */
  explicit PhysicsWorld(const PhysicsConfig& config = PhysicsConfig())
      : config_(config) {}

  /**
   * @brief Creates and registers a dynamic rigid body.
   * @param mass_kg Body mass in kilograms. Non-positive values are sanitized by @ref RigidBody.
   * @return Reference to the newly added body owned by this world.
   */
  RigidBody& createBody(double mass_kg);

  /**
   * @brief Creates and registers a rigid assembly container.
   * @return Reference to the newly added assembly owned by this world.
   */
  RigidAssembly& createAssembly();

  /**
   * @brief Adds a new environmental boundary primitive.
   * @return Reference to the appended boundary.
   *
   * Callers can then configure geometry, behavior, material id, and collision
   * filters directly through the returned reference.
   */
  EnvironmentalBoundary& addBoundary();

  /**
   * @brief Mutable access to boundary storage.
   * @return Reference to the internal boundary vector.
   * @warning Mutating the container while stepping is undefined at the API level.
   */
  std::vector<EnvironmentalBoundary>& boundaries() { return boundaries_; }

  /**
   * @brief Immutable access to boundary storage.
   * @return Const reference to the internal boundary vector.
   */
  const std::vector<EnvironmentalBoundary>& boundaries() const {
    return boundaries_;
  }

  /**
   * @brief Registers a world-level force generator.
   * @param generator Shared pointer to a force generator implementation.
   *
   * Force generators are evaluated during @ref step and can apply additional
   * forces to bodies independent of per-body material settings.
   */
  void addGlobalForceGenerator(
      const std::shared_ptr<ForceGenerator>& generator);

  /**
   * @brief Registers or updates one material interaction override row.
   * @param interaction Interaction record to insert or replace.
   */
  void setMaterialInteraction(const MaterialInteraction& interaction);

  /**
   * @brief Clears all material interaction override rows.
   *
   * After calling this, contact response falls back to default body/boundary
   * material coefficients.
   */
  void clearMaterialInteractions();

  /** @brief Mutable access to all rigid bodies currently registered in the world. */
  std::vector<RigidBody>& bodies() { return bodies_; }

  /** @brief Immutable access to all rigid bodies currently registered in the world. */
  const std::vector<RigidBody>& bodies() const { return bodies_; }

  /** @brief Mutable access to all rigid assemblies currently registered in the world. */
  std::vector<RigidAssembly>& assemblies() { return assemblies_; }

  /** @brief Immutable access to all rigid assemblies currently registered in the world. */
  const std::vector<RigidAssembly>& assemblies() const { return assemblies_; }

  /**
   * @brief Creates and registers a 3D ball simulator.
   * @param config Environment and behavior configuration for the ball simulator.
   * @param properties Physical properties for the simulated ball.
   * @return Reference to the newly added ball simulator.
   */
  BallPhysicsSim3D& createBall(
      const BallPhysicsSim3D::Config& config = BallPhysicsSim3D::Config(),
      const BallPhysicsSim3D::BallProperties& properties =
          BallPhysicsSim3D::BallProperties());

  /** @brief Mutable access to all registered ball simulators. */
  std::vector<BallPhysicsSim3D>& balls() { return balls_; }

  /** @brief Immutable access to all registered ball simulators. */
  const std::vector<BallPhysicsSim3D>& balls() const { return balls_; }

  /**
   * @brief Advances simulation by exactly one fixed timestep.
   *
   * The step procedure applies configured forces, integrates rigid-body states,
   * resolves supported contact responses, updates assemblies, and advances all
   * ball simulators. Internal counters for step count and simulated time are
   * incremented after successful completion.
   */
  void step();

  /** @brief Number of completed fixed-timestep iterations since construction/reset. */
  std::size_t stepCount() const { return step_count_; }

  /** @brief Total accumulated simulation time in seconds. */
  double accumulatedSimTimeS() const { return accumulated_sim_time_s_; }

  /**
   * @brief Mutable access to world configuration.
   * @return Reference to internal config object used by future steps.
   */
  PhysicsConfig& config() { return config_; }

  /**
   * @brief Read-only access to world configuration.
   * @return Const reference to internal config object.
   */
  const PhysicsConfig& config() const { return config_; }

  /**
   * @brief Sets world gravity acceleration from scalar components.
   * @param gx_mps2 Gravity x component in meters per second squared.
   * @param gy_mps2 Gravity y component in meters per second squared.
   * @param gz_mps2 Gravity z component in meters per second squared.
   *
   * Calling this method also enables gravity in @ref PhysicsConfig, making the
   * new vector effective for subsequent @ref step calls.
   */
  void setGravity(double gx_mps2, double gy_mps2, double gz_mps2) {
    config_.gravity_mps2 = Vector3{gx_mps2, gy_mps2, gz_mps2};
    config_.enable_gravity = true;
  }

  /**
   * @brief Reads world gravity acceleration into optional scalar outputs.
   * @param gx_mps2 Optional output pointer for gravity x in m/s^2.
   * @param gy_mps2 Optional output pointer for gravity y in m/s^2.
   * @param gz_mps2 Optional output pointer for gravity z in m/s^2.
   *
   * Any pointer may be null, allowing callers to request only selected axes.
   */
  void gravity(double* gx_mps2, double* gy_mps2, double* gz_mps2) const {
    if (gx_mps2) {
      *gx_mps2 = config_.gravity_mps2.x;
    }
    if (gy_mps2) {
      *gy_mps2 = config_.gravity_mps2.y;
    }
    if (gz_mps2) {
      *gz_mps2 = config_.gravity_mps2.z;
    }
  }

 private:
  /**
   * @brief Evaluates symmetric layer/mask eligibility between two colliders.
   * @return True when both directions of the bitmask test pass.
   */
  bool shouldInteract(std::uint32_t layer_a, std::uint32_t mask_a,
                      std::uint32_t layer_b, std::uint32_t mask_b) const;

  /**
   * @brief Looks up a material interaction override for an unordered pair.
   * @param material_a_id First material id.
   * @param material_b_id Second material id.
   * @return Pointer to matching override row when present and enabled; otherwise null.
   */
  const MaterialInteraction* findMaterialInteraction(
      std::int32_t material_a_id, std::int32_t material_b_id) const;

  PhysicsConfig config_{};

  std::vector<RigidBody> bodies_{};
  std::vector<RigidAssembly> assemblies_{};
  std::vector<BallPhysicsSim3D> balls_{};
  std::vector<EnvironmentalBoundary> boundaries_{};
  std::vector<std::shared_ptr<ForceGenerator>> global_force_generators_{};
  std::vector<MaterialInteraction> material_interactions_{};

  std::size_t step_count_{0};
  double accumulated_sim_time_s_{0.0};
};

}  // namespace frcsim
