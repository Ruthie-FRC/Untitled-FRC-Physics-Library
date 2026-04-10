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

/**
 * @brief Collision or constraint boundary definition for physics simulation.
 *
 * Models walls, planes, boxes, and cylinders with configurable interaction
 * behavior: rigid-body physics or faster static constraints. The normal() helper
 * is a placeholder for dynamic surface normal queries not yet fully generalized
 * across all geometry types.
 */
struct EnvironmentalBoundary {
  /** @brief Geometry type of this boundary primitive. */
  BoundaryType type{BoundaryType::kWall};
  /** @brief Physics interaction mode for this boundary. */
  BoundaryBehavior behavior{BoundaryBehavior::kStaticConstraint};

  /** @brief World-space position/center in meters. */
  Vector3 position_m{};
  /** @brief World-space orientation quaternion for rotated geometries. */
  Quaternion orientation{};

  /** @brief Half extents for box-like geometries in meters. */
  Vector3 half_extents_m{1.0, 1.0, 1.0};
  /** @brief Radius for cylindrical and spherical geometries in meters. */
  double radius_m{1.0};

  /** @brief Coefficient of restitution (bounce) in [0, 1]. */
  double restitution{0.5};
  /** @brief Tangential friction coefficient used in collision response. */
  double friction_coefficient{0.7};

  /** @brief User-defined tag for scenario-specific interaction logic. */
  int user_id{0};
  /** @brief Enables or disables collisions without removing the boundary. */
  bool is_active{true};

  /**
   * @brief Returns the default world-space normal vector (+Z up).
   * @return Const reference to a constant {0, 0, 1} unit vector.
   * @note TODO: Generalize to return orientation-dependent normals for all shape types.
   */
  const Vector3& normal() const {
    static const Vector3 default_normal(0.0, 0.0, 1.0);
    return default_normal;
  }
};

}  // namespace frcsim
