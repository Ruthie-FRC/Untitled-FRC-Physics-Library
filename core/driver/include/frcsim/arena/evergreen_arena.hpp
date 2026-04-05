#pragma once

#include "frcsim/arena/simulated_arena.hpp"
#include "frcsim/field/field_wall.hpp"

namespace frcsim {

class EvergreenArena : public SimulatedArena {
  public:
    EvergreenArena() : SimulatedArena(BallGamepieceSim::evergreenFieldConfig()) {
        gamepieceSim().configureEvergreenField();

        BallGamepieceSim::GamePieceInfo default_ball;
        default_ball.type = "Ball";
        default_ball.physics_config = BallGamepieceSim::defaultBallConfig();
        default_ball.ball_properties = BallGamepieceSim::defaultBallProperties();
        default_ball.spawn_on_ground_after_projectile = true;
        gamepieceSim().registerGamePieceType(default_ball);

        applyFieldMap(defaultFieldMap());
    }

    static FieldMap defaultFieldMap() {
        FieldMap map;

        auto walls = FieldWall::makeAxisAlignedPerimeter(
            BallGamepieceSim::evergreenFieldConfig().min_corner_m,
            BallGamepieceSim::evergreenFieldConfig().max_corner_m,
            1.0,
            0.25,
            0.6);
        for (const auto& wall : walls) {
            map.obstacles.push_back(wall);
        }

        GoalStructure center_goal;
        center_goal.shape = GoalStructure::Shape::kSphere;
        center_goal.center_m = Vector3(8.27, 4.10, 1.4);
        center_goal.radius_m = 0.55;
        center_goal.accepted_type = "Ball";
        map.goals.push_back(center_goal);

        return map;
    }
};

}  // namespace frcsim
