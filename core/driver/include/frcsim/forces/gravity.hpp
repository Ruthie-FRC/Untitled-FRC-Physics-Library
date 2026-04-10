// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/forces/force_generator.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

/**
 * @brief Force generator that applies constant gravity acceleration to a body.
 *
 * The gravitational force applied is mass × gravity_mps2, where gravity_mps2
 * is typically {0, 0, -9.81} m/s^2 for downward Earth gravity.
 */
class GravityForce : public ForceGenerator {
 public:
  /**
   * @brief Constructs gravity force generator.
   * @param gravity_mps2 Acceleration vector in m/s^2 (typically pointing downward).
   */
  explicit GravityForce(const Vector3& gravity_mps2)
      : gravity_mps2_(gravity_mps2) {}

  /**
   * @brief Applies gravitational force to a rigid body (mass × gravity).
   * @param body Rigid body to apply gravity to; kinematic bodies are skipped.
   * @param dt_s Timestep duration (unused).
   */
  void apply(RigidBody& body, double /*dt_s*/) const override {
    if (body.flags().is_kinematic)
      return;
    body.applyForce(gravity_mps2_ * body.massKg());
  }

 private:
  Vector3 gravity_mps2_{};
};

}  // namespace frcsim
