// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"

namespace frcsim {

/**
 * @brief Computes a Magnus lift force from linear velocity and spin.
 *
 * This model is intentionally small: it just wraps the shared
 * Vector3::magnusForce helper behind a configurable scalar coefficient so the
 * same lift formulation can be reused by game-piece, shot, and simulation code
 * without duplicating the cross-product math.
 */
class MagnusModel {
 public:
  /**
   * @brief Creates a Magnus-force model with the given coefficient.
   * @param magnus_coefficient Scalar applied to the omega x v lift term.
   *        The default matches the rest of the physics code and is tuned for
   *        SI inputs (meters per second and radians per second).
   */
  explicit MagnusModel(double magnus_coefficient = 1.0e-4)
      : magnus_coefficient_(magnus_coefficient) {}

  /** @brief Returns the current Magnus coefficient. */
  double magnusCoefficient() const { return magnus_coefficient_; }

  /**
   * @brief Updates the Magnus coefficient used by computeForce().
   * @param coefficient New scalar multiplier for the lift term.
   */
  void setMagnusCoefficient(double coefficient) {
    magnus_coefficient_ = coefficient;
  }

  /**
   * @brief Computes the lift force produced by the supplied velocity and spin.
   * @param velocity_mps Linear velocity vector in meters per second.
   * @param spin_radps Angular velocity vector in radians per second.
   *        Both vectors must be expressed in the same reference frame, because
   *        the result is the cross product omega x v scaled by the model
   *        coefficient.
   * @return Magnus force vector in newtons.
   */
  Vector3 computeForce(const Vector3& velocity_mps,
                       const Vector3& spin_radps) const {
    return Vector3::magnusForce(velocity_mps, spin_radps,
                                magnus_coefficient_);
  }

 private:
  double magnus_coefficient_{1.0e-4};
};

}  // namespace frcsim
