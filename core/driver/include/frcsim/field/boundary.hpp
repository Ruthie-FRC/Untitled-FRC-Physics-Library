// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <cstdint>

#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

/**
 * @brief Supported environmental boundary geometry kinds.
 *
 * These values describe the intended collision primitive semantics for
 * field geometry and world obstacles.
 */
enum class BoundaryType {
  /** @brief Finite wall-like barrier primitive. */
  kWall,
  /** @brief Infinite plane constraint primitive. */
  kPlane,
  /** @brief Box primitive (axis-aligned or oriented via quaternion). */
  kBox,
  /** @brief Cylindrical primitive (for posts, exclusion zones, etc.). */
  kCylinder,
};

/**
 * @brief Boundary interaction mode.
 *
 * Selects whether the boundary should participate as a richer rigid-body-like
 * contact target or as a lighter static constraint.
 */
enum class BoundaryBehavior {
  /** @brief Treat boundary as a rigid collision participant. */
  kRigidBody,
  /** @brief Treat boundary as a fast static constraint surface. */
  kStaticConstraint,
};

/**
 * @brief Collision or constraint boundary definition used by @ref PhysicsWorld.
 *
 * This structure captures geometry, transform, contact coefficients, and broad
 * filtering metadata for world boundaries. It is intentionally POD-like to keep
 * setup and serialization straightforward for JNI/FFI bridges.
 */
struct EnvironmentalBoundary {
  /** @brief Geometry primitive type that determines shape semantics. */
  BoundaryType type{BoundaryType::kWall};

  /** @brief Boundary interaction behavior mode. */
  BoundaryBehavior behavior{BoundaryBehavior::kStaticConstraint};

  /**
   * @brief World-space boundary origin/center in meters.
   *
   * Interpretation depends on @ref type (for example, plane anchor point versus
   * box/cylinder center).
   */
  Vector3 position_m{};

  /**
   * @brief World-space boundary orientation.
   *
   * Used for oriented primitives and for future generalized normal queries.
   */
  Quaternion orientation{};

  /**
   * @brief Half extents in meters for box-like boundaries.
   *
   * Components should be non-negative.
   */
  Vector3 half_extents_m{1.0, 1.0, 1.0};

  /**
   * @brief Radius in meters for cylindrical or radial primitives.
   *
   * Should be non-negative.
   */
  double radius_m{1.0};

  /**
   * @brief Coefficient of restitution (bounce response).
   *
   * Typical range is [0, 1], where 0 is fully inelastic and 1 is perfectly
   * elastic under simplified contact assumptions.
   */
  double restitution{0.5};

  /**
   * @brief Tangential friction coefficient for contact response.
   *
   * Values are typically >= 0. Higher values reduce tangential slip.
   */
  double friction_coefficient{0.7};

  /**
   * @brief Application-defined identifier for scenario-specific routing.
   *
   * Not interpreted by core physics logic.
   */
  int user_id{0};

  /**
   * @brief Numeric material id used by world material interaction lookup.
   */
  std::int32_t material_id{0};

  /**
   * @brief Broad-phase layer bitset for this boundary.
   *
   * Defaults to all bits set (interacts with all layers unless masked out).
   */
  std::uint32_t collision_layer_bits{0xFFFFFFFFu};

  /**
   * @brief Broad-phase mask bitset for this boundary.
   *
   * Defaults to all bits set (accepts all layers unless masked out).
   */
  std::uint32_t collision_mask_bits{0xFFFFFFFFu};

  /**
   * @brief Active flag for enabling/disabling participation without removal.
   */
  bool is_active{true};

  /**
   * @brief Returns a default world-space normal vector.
   * @return Const reference to a {0, 0, 1} unit vector.
   *
   * @note This is a placeholder helper. A future implementation can compute
   * orientation-aware normals per boundary geometry and contact point.
   */
  const Vector3& normal() const {
    static const Vector3 default_normal(0.0, 0.0, 1.0);
    return default_normal;
  }
};

}  // namespace frcsim
