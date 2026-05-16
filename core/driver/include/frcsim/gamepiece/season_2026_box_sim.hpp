// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/gamepiece/box_gamepiece_presets.hpp"

namespace frcsim {

/** @brief Compatibility wrapper around BoxPhysicsSim3D with 2026 default
 * presets. */
class Season2026BoxSim : public BoxPhysicsSim3D {
 public:
  /** @brief Inherit BoxPhysicsSim3D constructors. */
  using BoxPhysicsSim3D::BoxPhysicsSim3D;

  /**
   * @brief Returns default 2026 box geometry properties.
   * @return BoxProperties configured for example 2026 rectangular hitboxes.
   */
  static BoxPhysicsSim3D::BoxProperties defaultSeasonBoxProperties() {
    return BoxGamepiecePresets::season2026BoxProperties();
  }

  /**
   * @brief Returns default 2026 box environmental physics configuration.
   * @return BoxPhysicsSim3D::Config tuned for example 2026 simulation.
   */
  static BoxPhysicsSim3D::Config defaultSeasonBoxConfig() {
    return BoxGamepiecePresets::season2026BoxConfig();
  }
};

}  // namespace frcsim