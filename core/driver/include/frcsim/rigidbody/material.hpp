// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

namespace frcsim {

/**
 * @brief Contact material parameters for collision response.
 *
 * Physics principles: coefficient of restitution models normal impulse energy
 * retention; static/kinetic coefficients approximate Coulomb friction limits;
 * damping provides additional numerical dissipation after impacts.
 *
 * References:
 * https://en.wikipedia.org/wiki/Coefficient_of_restitution
 * https://en.wikipedia.org/wiki/Friction#Coulomb_friction
 */
struct Material {
  /// Coefficient of restitution (bounciness): 0 = absorb, 1 = perfectly elastic
  double coefficient_of_restitution{0.5};

  /// Coefficient of kinetic friction (sliding)
  double coefficient_of_friction_kinetic{0.6};

  /// Coefficient of static friction (stiction)
  double coefficient_of_friction_static{0.8};

  /// Damping multiplier applied to collisions (0 = no damping, 1 = full
  /// damping)
  double collision_damping{0.1};
};

}  // namespace frcsim
