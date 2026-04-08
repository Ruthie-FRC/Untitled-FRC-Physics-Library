// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

/** @brief Supported collision-boundary geometry kinds. */
enum class BoundaryType {
  kWall,      // Fixed planar barrier
  kPlane,     // Infinite plane constraint
  kBox,       // Axis-aligned or rotated box region
  kCylinder,  // Cylindrical region (e.g., exclusion zone)
};

/** @brief Boundary interaction mode for simulation. */
enum class BoundaryBehavior {
  kRigidBody,         // Treat as a moving or static rigid body with physics
                      // interactions
  kStaticConstraint,  // Enforce constraint without full physics interactions
                      // (faster)
};

/** @brief Generic boundary primitive used by gamepiece and arena simulation
 * layers. */
struct EnvironmentalBoundary {
  /** Geometry type of this boundary. */
  BoundaryType type{BoundaryType::kWall};
  /** Interaction behavior mode for this boundary. */
  BoundaryBehavior behavior{BoundaryBehavior::kStaticConstraint};

  /** World-space origin/center of the boundary. */
  Vector3 position_m{};
  /** World-space orientation for non-axis-aligned boundaries. */
  Quaternion orientation{};

  /** Half extents for box-like shapes. */
  Vector3 half_extents_m{1.0, 1.0, 1.0};
  /** Radius for cylindrical/spherical style boundaries. */
  double radius_m{1.0};

  /** Coefficient of restitution used by collision response. */
  double restitution{0.5};
  /** Tangential friction coefficient used by collision response. */
  double friction_coefficient{0.7};

  /** Optional user tag for scenario-specific logic. */
  int user_id{0};
  /** Enables or disables this boundary without removing it. */
  bool is_active{true};

  /**
   * @brief Returns a default world-up normal vector.
   * @return Reference to a constant +Z unit vector.
   */
  const Vector3& normal() const {
    static const Vector3 default_normal(0.0, 0.0, 1.0);
    return default_normal;
  }
};

}  // namespace frcsim
