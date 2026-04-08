// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/forces/force_generator.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

class GravityForce final : public ForceGenerator {
 public:
  explicit GravityForce(const Vector3& gravity_mps2 = Vector3{0.0, 0.0,
                                                              -9.80665})
      : gravity_mps2_(gravity_mps2) {}

  void setGravity(const Vector3& gravity_mps2) { gravity_mps2_ = gravity_mps2; }
  const Vector3& gravity() const { return gravity_mps2_; }

  void apply(RigidBody& body, double /*dt_s*/) const override {
    if (body.isStatic())
      return;
    body.applyForce(gravity_mps2_ * body.massKg());
  }

 private:
  Vector3 gravity_mps2_;
};

}  // namespace frcsim
