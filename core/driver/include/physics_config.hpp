// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"
#include "frcsim/math/quaternion.hpp"

namespace frcsim {

/**
 * @brief Configuration parameters for the physics world and simulation.
 */
struct PhysicsConfig {
  /** Fixed timestep for simulation updates, in seconds. */
  double fixed_dt_s{0.01};

  /** Gravity acceleration vector, in meters per second squared. */
  Vector3 gravity_mps2{0.0, 0.0, -9.81};

  /** Damping factors applied to linear and angular motion. */
  double linear_damping_per_s{0.0};
  double angular_damping_per_s{0.0};

  /** Integration method used for simulation updates. */
  enum class IntegrationMethod {
    kSemiImplicitEuler,
    kExplicitEuler,
    kRK2,
  } integration_method{IntegrationMethod::kSemiImplicitEuler};

  /** Enables collision detection between bodies. */
  bool enable_collision_detection{true};

  /** Enables joint constraint solving for assemblies. */
  bool enable_joint_constraints{true};

  /** Enables aerodynamic forces (drag and lift) on bodies. */
  bool enable_aerodynamics{false};

  /** Default drag coefficient used for aerodynamic force calculations. */
  double default_drag_coefficient{0.47};

  /** Default reference area for drag force calculations. */
  double default_drag_reference_area_m2{0.01};

  /** Air density used in aerodynamic force calculations. */
  double air_density_kgpm3{1.225};

  /** Magnus effect coefficient for spin-induced lift. */
  double magnus_coefficient{1.0e-4};

  /** Ground height used for ball-ground collision detection. */
  double ground_height_m{0.0};

  /** Rolling friction coefficient applied to balls on the ground. */
  double rolling_friction_per_s{1.5};

  /** Minimum bounce speed for balls to regain energy after a bounce. */
  double min_bounce_speed_mps{0.06};

  /** Baumgarte stabilization factor to reduce penetration in constraints. */
  double baumgarte_beta{0.2};

  /** Allowed penetration before Baumgarte stabilization is applied, in meters.
   */
  double baumgarte_slop_m{0.005};
};

}  // namespace frcsim