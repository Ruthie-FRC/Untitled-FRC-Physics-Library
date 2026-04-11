// Copyright (c) Jsim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/arena/simulated_arena.hpp"
#include "frcsim/field/field_wall.hpp"
#include "frcsim/gamepiece/ball_gamepiece_presets.hpp"

namespace frcsim {

/**
 * @brief Built-in arena preset representing the Evergreen benchmark field.
 *
 * This layout is intentionally static and is not tied to a single official FRC
 * season game manual. The geometry reflects the 2026 WCP CADathon "Evergreen"
 * concept, which served as a design exercise and was not physically built as an
 * official field.
 */
class EvergreenArena : public SimulatedArena {
 public:
  /** @brief Constructs Evergreen arena with default ball type registration and
   * map geometry. */
  EvergreenArena()
      : SimulatedArena(BallGamepiecePresets::evergreenFieldConfig()) {
    BallGamepieceSim::GamePieceInfo default_ball;
    default_ball.type = BallGamepieceSim::GamePieceType::kBall;
    default_ball.physics_config = BallGamepiecePresets::evergreenBallConfig();
    default_ball.ball_properties =
        BallGamepiecePresets::evergreenBallProperties();
    default_ball.spawn_on_ground_after_projectile = true;
    gamepieceSim().registerGamePieceType(default_ball);

    applyFieldMap(defaultFieldMap());
  }

  /** @brief Returns default Evergreen field obstacles and goal structures. */
  static FieldMap defaultFieldMap() {
    FieldMap map;

    auto walls = FieldWall::makeAxisAlignedPerimeter(
        BallGamepiecePresets::evergreenFieldConfig().min_corner_m,
        BallGamepiecePresets::evergreenFieldConfig().max_corner_m, 1.0, 0.25,
        0.6);
    for (const auto& wall : walls) {
      map.obstacles.push_back(wall);
    }

    GoalStructure center_goal;
    center_goal.shape = GoalStructure::Shape::kSphere;
    center_goal.center_m = Vector3(8.27, 4.10, 1.4);
    center_goal.radius_m = 0.55;
    center_goal.accepted_type = GoalStructure::AcceptedType::kBall;
    map.goals.push_back(center_goal);

    return map;
  }
};

}  // namespace frcsim
