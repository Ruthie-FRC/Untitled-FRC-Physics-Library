#pragma once

#include "frcsim/joints/joint_base.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

class RigidBody;

class FixedJoint : public JointBase {
  public:
    FixedJoint(RigidBody* body_a, RigidBody* body_b)
        : JointBase(JointType::kFixed, body_a, body_b) {}

    void solveConstraint(double dt_s, int iterations) override;
    double constraintError() const override;

  private:
    Vector3 cached_impulse_linear_{};
    Vector3 cached_impulse_angular_{};
};

}  // namespace frcsim
