// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <functional>

#include "frcsim/math/vector.hpp"

namespace frcsim {

/** @brief Scoring volume definition used by arena presets and projectile
 * checks. */
struct GoalStructure {
  /** @brief Built-in geometry modes for goal containment tests. */
  enum class Shape {
    kBox,
    kSphere,
    /**
     * @brief User-defined shape handled by custom_position_checker.
     *
     * To use this mode, set shape to kCustom and provide
     * custom_position_checker.
     * @example
     * \code{.cpp}
     * GoalStructure goal{};
     * goal.shape = GoalStructure::Shape::kCustom;
     * goal.center_m = Vector3(5.0, 2.0, 1.6);
     * goal.custom_position_checker = [c = goal.center_m](const Vector3& p) {
     *     // Example: accept points inside a simple vertical "capsule-like"
     * scoring zone. const Vector3 d = p - c; const double radial =
     * std::sqrt(d.x * d.x + d.y * d.y); return radial <= 0.35 && d.z >= -0.25
     * && d.z <= 0.45;
     * };
     * \endcode
     */
    kCustom,
  };

  /** @brief Supported gamepiece type filters for scoring validation. */
  enum class AcceptedType {
    kAny,
    kBall,
    kCustom1,
    kCustom2,
    kCustom3,
    kCustom4,
  };

  /** Selected geometry mode for this goal. */
  Shape shape{Shape::kBox};
  /** @brief Goal center position in world coordinates (meters). */
  Vector3 center_m{};
  /** @brief Half-extent dimensions used when shape is kBox (meters). */
  Vector3 half_extents_m{0.2, 0.2, 0.2};
  /** @brief Sphere radius used when shape is kSphere (meters). */
  double radius_m{0.25};

  /** @brief Filter for accepted gamepiece types (allows selective scoring). */
  AcceptedType accepted_type{AcceptedType::kBall};
  /** @brief When true, scoring requires upward (+Z) velocity unless custom validator overrides. */
  bool require_positive_vertical_velocity{false};

  /** @brief Optional custom position test for complex geometry in kCustom mode. */
  std::function<bool(const Vector3&)> custom_position_checker{};

  /** @brief Optional custom validator for velocity constraints during scoring. */
  std::function<bool(const Vector3&)> custom_velocity_validator{};

  /**
   * @brief Test whether a position is contained within this goal's volume.
   * @param position_m Position to test in world coordinates (meters).
   * @return True when position is inside the goal geometry.
   */
  bool contains(const Vector3& position_m) const {
    if (shape == Shape::kCustom && custom_position_checker) {
      return custom_position_checker(position_m);
    }

    if (shape == Shape::kSphere) {
      return (position_m - center_m).norm() <= radius_m;
    }

    const Vector3 delta = position_m - center_m;
    return std::abs(delta.x) <= half_extents_m.x &&
           std::abs(delta.y) <= half_extents_m.y &&
           std::abs(delta.z) <= half_extents_m.z;
  }

  /**
   * @brief Test whether a velocity satisfies score acceptance constraints.
   * @param velocity_mps Velocity vector in world frame (m/s).
   * @return True when velocity meets configured requirements.
   */
  bool velocityAllowed(const Vector3& velocity_mps) const {
    if (custom_velocity_validator) {
      return custom_velocity_validator(velocity_mps);
    }
    if (require_positive_vertical_velocity) {
      return velocity_mps.z > 0.0;
    }
    return true;
  }
};

}  // namespace frcsim
