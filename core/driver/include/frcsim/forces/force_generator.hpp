// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

namespace frcsim {

class RigidBody;

class ForceGenerator {
 public:
  virtual ~ForceGenerator() = default;
  virtual void apply(RigidBody& body, double dt_s) const = 0;
};

}  // namespace frcsim
