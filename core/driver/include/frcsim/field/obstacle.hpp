// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <vector>

#include "frcsim/field/boundary.hpp"

namespace frcsim {

/**
 * @brief Convenience wrapper for constructing EnvironmentalBoundary-based
 * obstacles.
 */
struct FieldObstacle {
  /** Underlying boundary object consumed by BallGamepieceSim. */
  EnvironmentalBoundary boundary{};

  /**
   * @brief Creates an infinite plane obstacle.
   * @param point_m A point on the plane in world coordinates.
   * @param orientation Plane orientation; local +Z is treated as outward
   * normal.
   * @param restitution Normal restitution coefficient.
   *        Default is 0.3, chosen as a conservative "not perfectly rigid" value
   * for common field elements (wood, polycarbonate, carpet transitions). Lower
   * values reduce bounce and help objects settle; higher values increase
   *        rebound speed and can make trajectories livelier but less damped.
   * @param friction Tangential friction coefficient.
   *        Default is 0.6, which approximates moderate grip so glancing
   * contacts lose tangential speed without unrealistically sticking. Lower
   * values permit more sliding; higher values increase scrub/rolling slowdown.
   * @param user_id Optional tag for custom behavior grouping.
   *        Default is 0 (no special grouping). Use non-zero IDs to classify
   * boundaries for custom contact logic, filtering, analytics, or game-specific
   * rules.
   */
  static FieldObstacle makePlane(const Vector3& point_m,
                                 const Quaternion& orientation,
                                 double restitution = 0.3,
                                 double friction = 0.6, int user_id = 0) {
    FieldObstacle obstacle{};
    obstacle.boundary.type = BoundaryType::kPlane;
    obstacle.boundary.position_m = point_m;
    obstacle.boundary.orientation = orientation;
    obstacle.boundary.restitution = restitution;
    obstacle.boundary.friction_coefficient = friction;
    obstacle.boundary.user_id = user_id;
    return obstacle;
  }

  /**
   * @brief Creates an oriented box obstacle.
   * @param center_m Box center in world coordinates.
   * @param half_extents_m Positive half-lengths along local X/Y/Z axes.
   * @param orientation Box orientation quaternion.
   * @param restitution Normal restitution coefficient.
   *        Default 0.3 keeps collisions plausibly inelastic for most FRC-style
   * field interactions while still allowing visible bounce in high-speed
   * impacts.
   * @param friction Tangential friction coefficient.
   *        Default 0.6 gives balanced tangential damping (less endless sliding
   * along walls, without overly abrupt sticking in grazing contacts).
   * @param user_id Optional tag for custom behavior grouping.
   *        Default 0 means "unclassified"; assign other values when you need to
   *        identify this obstacle in custom behavior or telemetry code.
   *
   * Quaternion note: use Quaternion::fromEuler(roll, pitch, yaw) to rotate away
   * from axis alignment.
   */
  static FieldObstacle makeBox(const Vector3& center_m,
                               const Vector3& half_extents_m,
                               const Quaternion& orientation = Quaternion(),
                               double restitution = 0.3, double friction = 0.6,
                               int user_id = 0) {
    FieldObstacle obstacle{};
    obstacle.boundary.type = BoundaryType::kBox;
    obstacle.boundary.position_m = center_m;
    obstacle.boundary.half_extents_m = half_extents_m;
    obstacle.boundary.orientation = orientation;
    obstacle.boundary.restitution = restitution;
    obstacle.boundary.friction_coefficient = friction;
    obstacle.boundary.user_id = user_id;
    return obstacle;
  }

  /**
   * @brief Creates an oriented cylinder obstacle aligned to local Z.
   * @param center_m Cylinder center in world coordinates.
   * @param radius_m Cylinder radius.
   * @param half_height_m Half of cylinder height along local Z.
   * @param orientation Cylinder orientation quaternion.
   * @param restitution Normal restitution coefficient.
   *        Default 0.3 for the same reason as other boundary factories:
   * generally damped contacts that still show rebound at speed.
   * @param friction Tangential friction coefficient.
   *        Default 0.6 to represent typical non-ice, non-rubberized field
   * contact behavior.
   * @param user_id Optional tag for custom behavior grouping.
   *        Default 0; set to a custom ID when this cylinder must be
   * distinguished from other obstacles during contact handling.
   *
   * Example:
   * \code{.cpp}
   * auto q = Quaternion::fromEuler(0.0, 0.0, 1.57079632679); // rotate 90 deg
   * about Z auto post = FieldObstacle::makeCylinder(Vector3(2.0, 1.0, 0.6),
   * 0.15, 0.6, q);
   * \endcode
   */
  static FieldObstacle makeCylinder(
      const Vector3& center_m, double radius_m, double half_height_m,
      const Quaternion& orientation = Quaternion(), double restitution = 0.3,
      double friction = 0.6, int user_id = 0) {
    FieldObstacle obstacle{};
    obstacle.boundary.type = BoundaryType::kCylinder;
    obstacle.boundary.position_m = center_m;
    obstacle.boundary.radius_m = radius_m;
    obstacle.boundary.half_extents_m = Vector3(0.0, 0.0, half_height_m);
    obstacle.boundary.orientation = orientation;
    obstacle.boundary.restitution = restitution;
    obstacle.boundary.friction_coefficient = friction;
    obstacle.boundary.user_id = user_id;
    return obstacle;
  }
};

/** @brief Mutable collection helper for assembling obstacle sets. */
struct FieldObstacleMap {
  /** Stored obstacle entries. */
  std::vector<FieldObstacle> obstacles{};

  /** @brief Appends a single obstacle. */
  void add(const FieldObstacle& obstacle) { obstacles.push_back(obstacle); }
};

}  // namespace frcsim
