// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/joints/joint_base.hpp"

namespace frcsim {

class FixedJoint : public JointBase {
 public:
  FixedJoint(RigidBody* body_a, RigidBody* body_b)
      : JointBase(JointType::kFixed, body_a, body_b) {}

  void solveConstraint(double dt_s, int iterations) override;
  double constraintError() const override;
};

}  // namespace frcsim
