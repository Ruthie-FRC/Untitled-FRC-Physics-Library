// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"

namespace frcsim {

class MagnusModel {
 public:
  explicit MagnusModel(double magnus_coefficient = 1.0e-4)
      : magnus_coefficient_(magnus_coefficient) {}

  double magnusCoefficient() const { return magnus_coefficient_; }
  void setMagnusCoefficient(double coefficient) {
    magnus_coefficient_ = coefficient;
  }

  /**
   * @brief Computes the lift force produced by the supplied velocity and spin.
   * @param velocity_mps Linear velocity vector in meters per second, expressed
   *     in the world frame.
   * @param spin_radps Angular velocity vector in radians per second, expressed
   *     in the same frame as velocity_mps.
   *     Both vectors must be expressed in the same reference frame.
   *     The returned force uses the model F = k * (omega x v), where k is
   *     magnusCoefficient().
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
