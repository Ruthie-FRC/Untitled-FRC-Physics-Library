#pragma once

#include <array>

#include "frcsim/field/obstacle.hpp"

namespace frcsim {

struct FieldWall {
    static std::array<FieldObstacle, 4> makeAxisAlignedPerimeter(const Vector3& min_corner_m,
                                                                 const Vector3& max_corner_m,
                                                                 double wall_height_m,
                                                                 double restitution = 0.25,
                                                                 double friction = 0.6) {
        const double mid_x = 0.5 * (min_corner_m.x + max_corner_m.x);
        const double mid_y = 0.5 * (min_corner_m.y + max_corner_m.y);
        const double half_x = 0.5 * (max_corner_m.x - min_corner_m.x);
        const double half_y = 0.5 * (max_corner_m.y - min_corner_m.y);
        const double half_h = std::max(0.05, 0.5 * wall_height_m);

        return {
            FieldObstacle::makeBox(Vector3(mid_x, min_corner_m.y, half_h), Vector3(half_x, 0.05, half_h), Quaternion(), restitution, friction),
            FieldObstacle::makeBox(Vector3(mid_x, max_corner_m.y, half_h), Vector3(half_x, 0.05, half_h), Quaternion(), restitution, friction),
            FieldObstacle::makeBox(Vector3(min_corner_m.x, mid_y, half_h), Vector3(0.05, half_y, half_h), Quaternion(), restitution, friction),
            FieldObstacle::makeBox(Vector3(max_corner_m.x, mid_y, half_h), Vector3(0.05, half_y, half_h), Quaternion(), restitution, friction),
        };
    }
};

}  // namespace frcsim
