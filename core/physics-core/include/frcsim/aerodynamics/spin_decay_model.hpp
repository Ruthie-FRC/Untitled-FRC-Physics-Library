// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim {

class SpinDecayModel {
 public:
  explicit SpinDecayModel(double decay_torque_gain = 1e-4)
      : decay_torque_gain_(decay_torque_gain) {}

  Vector3 computeTorque(const Vector3& spin_radps) const {
    return spin_radps * (-decay_torque_gain_);
  }

  void apply(RigidBody& body) const {
    if (body.isStatic())
      return;
    body.applyTorque(computeTorque(body.angularVelocity()));
  }

 private:
  double decay_torque_gain_{1e-4};
};

}  // namespace frcsim
