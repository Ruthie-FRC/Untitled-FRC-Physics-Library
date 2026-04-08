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

  Vector3 computeForce(const Vector3& velocity_mps,
                       const Vector3& spin_radps) const {
    return spin_radps.cross(velocity_mps) * magnus_coefficient_;
  }

 private:
  double magnus_coefficient_{1.0e-4};
};

}  // namespace frcsim
