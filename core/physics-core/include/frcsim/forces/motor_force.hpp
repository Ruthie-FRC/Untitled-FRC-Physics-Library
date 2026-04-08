// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <algorithm>

#include "frcsim/forces/force_generator.hpp"
#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

class MotorForce final : public ForceGenerator {
 public:
  MotorForce(double max_force_n, Vector3 local_direction = Vector3::unitX())
      : max_force_n_(std::max(0.0, max_force_n)),
        local_direction_(local_direction.normalized()) {}

  void setCommand(double normalized_command) {
    command_ = std::clamp(normalized_command, -1.0, 1.0);
  }

  void apply(RigidBody& body, double /*dt_s*/) const override {
    if (body.isStatic())
      return;
    const Vector3 world_direction =
        body.orientation().rotate(local_direction_).normalized();
    body.applyForce(world_direction * (max_force_n_ * command_));
  }

 private:
  double max_force_n_{0.0};
  double command_{0.0};
  Vector3 local_direction_{Vector3::unitX()};
};

}  // namespace frcsim
