// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>

#include "frcsim/field/obstacle.hpp"

namespace frcsim {

/**
 * @brief Helper factory for perimeter wall obstacles.
 *
 * Coordinate conventions follow WPILib field coordinates (blue-alliance
 * origin):
 * - Origin is at the blue-alliance right-corner wall intersection when viewed
 * from above.
 * - +X points from blue toward red downfield.
 * - +Y points across the field width.
 * - +Z points upward.
 */
struct FieldWall {
  /**
   * @brief Creates four axis-aligned box walls around a rectangular field
   * footprint.
   * @param min_corner_m Lower world-space corner of the field bounds.
   * @param max_corner_m Upper world-space corner of the field bounds.
   * @param wall_height_m Wall height in meters.
   * @param restitution Wall restitution for collisions.
   *        Default is 0.25, which models mostly energy-absorbing perimeter
   * walls so balls lose speed on impact instead of ping-ponging around the
   * field. Lower values (near 0) make impacts deaden quickly; higher values
   * (near 1) preserve more normal velocity and increase bounce height/distance.
   * @param friction Wall friction coefficient for tangential damping.
   *        Default is 0.6 to provide noticeable sliding resistance at contact.
   *        This balances "roll and settle" behavior versus prolonged
   * wall-skimming.
   */
  static std::array<FieldObstacle, 4> makeAxisAlignedPerimeter(
      const Vector3& min_corner_m, const Vector3& max_corner_m,
      double wall_height_m, double restitution = 0.25, double friction = 0.6) {
    const double mid_x = 0.5 * (min_corner_m.x + max_corner_m.x);
    const double mid_y = 0.5 * (min_corner_m.y + max_corner_m.y);
    const double half_x = 0.5 * (max_corner_m.x - min_corner_m.x);
    const double half_y = 0.5 * (max_corner_m.y - min_corner_m.y);
    const double half_h = std::max(0.05, 0.5 * wall_height_m);

    return {
        FieldObstacle::makeBox(Vector3(mid_x, min_corner_m.y, half_h),
                               Vector3(half_x, 0.05, half_h), Quaternion(),
                               restitution, friction),
        FieldObstacle::makeBox(Vector3(mid_x, max_corner_m.y, half_h),
                               Vector3(half_x, 0.05, half_h), Quaternion(),
                               restitution, friction),
        FieldObstacle::makeBox(Vector3(min_corner_m.x, mid_y, half_h),
                               Vector3(0.05, half_y, half_h), Quaternion(),
                               restitution, friction),
        FieldObstacle::makeBox(Vector3(max_corner_m.x, mid_y, half_h),
                               Vector3(0.05, half_y, half_h), Quaternion(),
                               restitution, friction),
    };
  }
};

}  // namespace frcsim
