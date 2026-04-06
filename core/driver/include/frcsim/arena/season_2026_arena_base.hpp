#pragma once

#include "frcsim/arena/simulated_arena.hpp"

namespace frcsim {

class Season2026ArenaBase : public SimulatedArena {
  public:
    Season2026ArenaBase() : SimulatedArena(seasonFieldConfig()) {
        gamepieceSim().configureSeason2026Field();
        gamepieceSim().registerGamePieceType(defaultBallGamePiece());
    }

    static const BallGamepieceSim::FieldConfig& seasonFieldConfig() {
        static const BallGamepieceSim::FieldConfig config = BallGamepieceSim::season2026FieldConfig();
        return config;
    }

    static BallGamepieceSim::GamePieceInfo defaultBallGamePiece() {
        BallGamepieceSim::GamePieceInfo season_ball;
        season_ball.type = "Ball";
        season_ball.physics_config = BallGamepieceSim::season2026BallConfig();
        season_ball.ball_properties = BallGamepieceSim::season2026BallProperties();
        season_ball.spawn_on_ground_after_projectile = true;
        return season_ball;
    }
};

}  // namespace frcsim