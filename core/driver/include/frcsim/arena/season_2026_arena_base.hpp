// Copyright (c) Jsim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/arena/simulated_arena.hpp"
#include "frcsim/gamepiece/ball_gamepiece_presets.hpp"

namespace frcsim {

/**
 * @brief Common base arena for 2026-season style field configuration and
 * default ball setup.
 */
class Season2026ArenaBase : public SimulatedArena {
 public:
  /** @brief Constructs the base arena and registers default 2026
   * field/gamepiece configuration. */
  Season2026ArenaBase() : SimulatedArena(seasonFieldConfig()) {
    BallGamepiecePresets::configureSeason2026Field(gamepieceSim());
    gamepieceSim().registerGamePieceType(defaultBallGamePiece());
  }

  /**
   * @brief Returns cached field configuration for 2026 season presets.
   * @return Immutable reference to reusable 2026 field configuration.
   */
  static const BallGamepieceSim::FieldConfig& seasonFieldConfig() {
    static const BallGamepieceSim::FieldConfig config =
        BallGamepiecePresets::season2026FieldConfig();
    return config;
  }

  /**
   * @brief Builds default gamepiece registration for 2026 season balls.
   * @return GamePieceInfo configured with season ball physics and type name.
   */
  static BallGamepieceSim::GamePieceInfo defaultBallGamePiece() {
    BallGamepieceSim::GamePieceInfo season_ball;
    season_ball.type = BallGamepieceSim::GamePieceType::kBall;
    season_ball.physics_config = BallGamepiecePresets::season2026BallConfig();
    season_ball.ball_properties =
        BallGamepiecePresets::season2026BallProperties();
    season_ball.spawn_on_ground_after_projectile = true;
    return season_ball;
  }
};

}  // namespace frcsim
