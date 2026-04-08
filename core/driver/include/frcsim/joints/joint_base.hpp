// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"

namespace frcsim {

class RigidBody;

enum class JointType {
  kRevolute,   // Hinge: rotation about an axis
  kPrismatic,  // Slider: translation along an axis
  kFixed,      // Rigid connection
  kBall,       // Ball joint: unconstrained rotation
  kDistance,   // Distance constraint between bodies
};

class JointBase {
 public:
  virtual ~JointBase() = default;

  JointType type() const { return type_; }
  RigidBody* bodyA() { return body_a_; }
  RigidBody* bodyB() { return body_b_; }
  const RigidBody* bodyA() const { return body_a_; }
  const RigidBody* bodyB() const { return body_b_; }

  // Anchor points in local coordinates of each body
  const Vector3& anchorA() const { return anchor_a_; }
  const Vector3& anchorB() const { return anchor_b_; }
  void setAnchorA(const Vector3& anchor) { anchor_a_ = anchor; }
  void setAnchorB(const Vector3& anchor) { anchor_b_ = anchor; }

  bool isEnabled() const { return is_enabled_; }
  void setEnabled(bool enabled) { is_enabled_ = enabled; }

  double breakForceThreshold() const { return break_force_threshold_; }
  void setBreakForceThreshold(double force_n) {
    break_force_threshold_ = force_n;
  }

  bool isBroken() const { return is_broken_; }
  void resetBroken() { is_broken_ = false; }

  // Main constraint solving interface (to be overridden per joint type)
  virtual void solveConstraint(double dt_s, int iterations) = 0;

  // Warm-start cached impulses from previous frame
  virtual void warmStart() {}

  // Compute constraint error for diagnostics
  virtual double constraintError() const = 0;

 protected:
  explicit JointBase(JointType type, RigidBody* body_a, RigidBody* body_b)
      : type_(type), body_a_(body_a), body_b_(body_b) {}

  JointType type_;
  RigidBody* body_a_{nullptr};
  RigidBody* body_b_{nullptr};
  Vector3 anchor_a_{};
  Vector3 anchor_b_{};
  bool is_enabled_{true};
  bool is_broken_{false};
  double break_force_threshold_{1e6};
};

}  // namespace frcsim
