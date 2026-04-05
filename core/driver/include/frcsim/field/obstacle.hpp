#pragma once

#include <vector>

#include "frcsim/field/boundary.hpp"

namespace frcsim {

struct FieldObstacle {
    EnvironmentalBoundary boundary{};

    static FieldObstacle makePlane(const Vector3& point_m,
                                   const Quaternion& orientation,
                                   double restitution = 0.3,
                                   double friction = 0.6,
                                   int user_id = 0) {
        FieldObstacle obstacle{};
        obstacle.boundary.type = BoundaryType::kPlane;
        obstacle.boundary.position_m = point_m;
        obstacle.boundary.orientation = orientation;
        obstacle.boundary.restitution = restitution;
        obstacle.boundary.friction_coefficient = friction;
        obstacle.boundary.user_id = user_id;
        return obstacle;
    }

    static FieldObstacle makeBox(const Vector3& center_m,
                                 const Vector3& half_extents_m,
                                 const Quaternion& orientation = Quaternion(),
                                 double restitution = 0.3,
                                 double friction = 0.6,
                                 int user_id = 0) {
        FieldObstacle obstacle{};
        obstacle.boundary.type = BoundaryType::kBox;
        obstacle.boundary.position_m = center_m;
        obstacle.boundary.half_extents_m = half_extents_m;
        obstacle.boundary.orientation = orientation;
        obstacle.boundary.restitution = restitution;
        obstacle.boundary.friction_coefficient = friction;
        obstacle.boundary.user_id = user_id;
        return obstacle;
    }

    static FieldObstacle makeCylinder(const Vector3& center_m,
                                      double radius_m,
                                      double half_height_m,
                                      const Quaternion& orientation = Quaternion(),
                                      double restitution = 0.3,
                                      double friction = 0.6,
                                      int user_id = 0) {
        FieldObstacle obstacle{};
        obstacle.boundary.type = BoundaryType::kCylinder;
        obstacle.boundary.position_m = center_m;
        obstacle.boundary.radius_m = radius_m;
        obstacle.boundary.half_extents_m = Vector3(0.0, 0.0, half_height_m);
        obstacle.boundary.orientation = orientation;
        obstacle.boundary.restitution = restitution;
        obstacle.boundary.friction_coefficient = friction;
        obstacle.boundary.user_id = user_id;
        return obstacle;
    }
};

struct FieldObstacleMap {
    std::vector<FieldObstacle> obstacles{};

    void add(const FieldObstacle& obstacle) {
        obstacles.push_back(obstacle);
    }
};

}  // namespace frcsim
