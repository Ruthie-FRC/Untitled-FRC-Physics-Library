// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/joints/joint_base.hpp"

namespace frcsim {

class PrismaticJoint : public JointBase {
 public:
  PrismaticJoint(RigidBody* body_a, RigidBody* body_b,
                 const Vector3& axis_local)
      : JointBase(JointType::kPrismatic, body_a, body_b),
        axis_local_(axis_local.normalized()) {}

  const Vector3& axisLocal() const { return axis_local_; }

  void setLimits(double min_displacement_m, double max_displacement_m) {
    has_limits_ = true;
    min_displacement_m_ = min_displacement_m;
    max_displacement_m_ = max_displacement_m;
  }
  bool hasLimits() const { return has_limits_; }
  double minDisplacement() const { return min_displacement_m_; }
  double maxDisplacement() const { return max_displacement_m_; }

  void setMotorTarget(double target_velocity_mps, double max_force_n) {
    has_motor_ = true;
    motor_target_velocity_mps_ = target_velocity_mps;
    motor_max_force_n_ = max_force_n;
  }
  bool hasMotor() const { return has_motor_; }
  double motorTargetVelocity() const { return motor_target_velocity_mps_; }
  double motorMaxForce() const { return motor_max_force_n_; }

  void solveConstraint(double dt_s, int iterations) override;
  double constraintError() const override;

 private:
  Vector3 axis_local_{Vector3::unitX()};

  bool has_limits_{false};
  double min_displacement_m_{0.0};
  double max_displacement_m_{0.0};

  bool has_motor_{false};
  double motor_target_velocity_mps_{0.0};
  double motor_max_force_n_{0.0};
};

}  // namespace frcsim
