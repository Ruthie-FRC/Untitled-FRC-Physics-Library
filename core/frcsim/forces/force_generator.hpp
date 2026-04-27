// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

namespace frcsim {

class RigidBody;

/**
 * @brief Interface for force laws used by the rigid-body integrator.
 *
 * Implementations encode a physics principle that maps the current body state
 * (and optionally time step) to an applied force in newtons. The world update
 * then uses Newton's second law, F = m a, to convert those forces into
 * accelerations.
 *
 * Reference:
 * https://en.wikipedia.org/wiki/Newton%27s_laws_of_motion
 */
class ForceGenerator {
 public:
  virtual ~ForceGenerator() = default;
  virtual void apply(RigidBody& body, double dt_s) const = 0;
};

}  // namespace frcsim
