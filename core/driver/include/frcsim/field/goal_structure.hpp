// Copyright (c) RenSim contributors.
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
  /** Goal center in world coordinates. */
  Vector3 center_m{};
  /** Half extents used when shape is kBox. */
  Vector3 half_extents_m{0.2, 0.2, 0.2};
  /** Radius used when shape is kSphere. */
  double radius_m{0.25};

  /** Accepted gamepiece type enum. */
  AcceptedType accepted_type{AcceptedType::kBall};
  /** If true, scoring is valid only for upward-moving objects unless overridden
   * by custom validator. */
  bool require_positive_vertical_velocity{false};

  /** Optional custom checker for complex scoring geometry in kCustom mode. */
  std::function<bool(const Vector3&)> custom_position_checker{};

  /** Optional custom validator for velocity constraints at score time. */
  std::function<bool(const Vector3&)> custom_velocity_validator{};

  /** @brief Returns true when position is inside this goal's geometry. */
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

  /** @brief Returns true when velocity satisfies configured score constraints.
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
