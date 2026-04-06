#pragma once

#include "frcsim/arena/season_2026_arena_base.hpp"
#include "frcsim/field/field_wall.hpp"

namespace frcsim {

class Rebuilt2026Arena : public Season2026ArenaBase {
  public:
        Rebuilt2026Arena() : Season2026ArenaBase() {
        applyFieldMap(rebuiltFieldMap());

        Timings t;
        t.robot_period_s = 0.02;
        t.simulation_subticks_per_period = 5;
        setTimings(t);
    }

    static FieldMap rebuiltFieldMap() {
        FieldMap map;

        auto walls = FieldWall::makeAxisAlignedPerimeter(
            seasonFieldConfig().min_corner_m,
            seasonFieldConfig().max_corner_m,
            1.0,
            0.22,
            0.65);
        for (const auto& wall : walls) {
            map.obstacles.push_back(wall);
        }

        // Example center structure + depots inspired by 2026 rebuilt arena structure.
        map.obstacles.push_back(FieldObstacle::makeBox(
            Vector3(7.35, 1.72, 0.5),
            Vector3(0.35, 0.25, 0.5),
            Quaternion(),
            0.2,
            0.7,
            2601));

        map.obstacles.push_back(FieldObstacle::makeBox(
            Vector3(0.35, 5.53, 0.5),
            Vector3(0.25, 0.35, 0.5),
            Quaternion(),
            0.2,
            0.7,
            2602));

        GoalStructure rebuilt_hub;
        rebuilt_hub.shape = GoalStructure::Shape::kBox;
        rebuilt_hub.center_m = Vector3(7.0, 4.1, 1.7);
        rebuilt_hub.half_extents_m = Vector3(0.4, 0.4, 0.5);
        rebuilt_hub.accepted_type = "Ball";
        map.goals.push_back(rebuilt_hub);

        GoalStructure outpost_goal;
        outpost_goal.shape = GoalStructure::Shape::kSphere;
        outpost_goal.center_m = Vector3(12.4, 4.1, 1.3);
        outpost_goal.radius_m = 0.45;
        outpost_goal.accepted_type = "Ball";
        map.goals.push_back(outpost_goal);

        return map;
    }
};

}  // namespace frcsim
