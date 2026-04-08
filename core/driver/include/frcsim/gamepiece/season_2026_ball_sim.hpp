// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/gamepiece/ball_gamepiece_presets.hpp"

namespace frcsim {

/** @brief Compatibility wrapper around BallGamepieceSim with 2026 default
 * presets. */
class Season2026BallSim : public BallGamepieceSim {
 public:
  /** @brief Inherit BallGamepieceSim constructors. */
  using BallGamepieceSim::BallGamepieceSim;

  /**
   * @brief Returns default 2026 ball material/geometry properties.
   * @return BallProperties configured for season 2026 game pieces.
   */
  static BallPhysicsSim3D::BallProperties defaultSeasonBallProperties() {
    return BallGamepiecePresets::season2026BallProperties();
  }

  /**
   * @brief Returns default 2026 ball environmental physics configuration.
   * @return BallPhysicsSim3D::Config tuned for season 2026 simulation.
   */
  static BallPhysicsSim3D::Config defaultSeasonBallConfig() {
    return BallGamepiecePresets::season2026BallConfig();
  }
};

}  // namespace frcsim
