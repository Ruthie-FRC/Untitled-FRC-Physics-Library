#pragma once

#include <functional>
#include <string>

#include "frcsim/math/vector.hpp"

namespace frcsim {

struct GoalStructure {
    enum class Shape {
        kBox,
        kSphere,
        kCustom,
    };

    Shape shape{Shape::kBox};
    Vector3 center_m{};
    Vector3 half_extents_m{0.2, 0.2, 0.2};
    double radius_m{0.25};

    std::string accepted_type{"Ball"};
    bool require_positive_vertical_velocity{false};

    // Optional custom checker for complex scoring geometry.
    std::function<bool(const Vector3&)> custom_position_checker{};

    // Optional custom validator for velocity constraints at score time.
    std::function<bool(const Vector3&)> custom_velocity_validator{};

    bool contains(const Vector3& position_m) const {
        if (shape == Shape::kCustom && custom_position_checker) {
            return custom_position_checker(position_m);
        }

        if (shape == Shape::kSphere) {
            return (position_m - center_m).norm() <= radius_m;
        }

        const Vector3 delta = position_m - center_m;
        return std::abs(delta.x) <= half_extents_m.x &&
               std::abs(delta.y) <= half_extents_m.y &&
               std::abs(delta.z) <= half_extents_m.z;
    }

    bool velocityAllowed(const Vector3& velocity_mps) const {
        if (custom_velocity_validator) {
            return custom_velocity_validator(velocity_mps);
        }
        if (require_positive_vertical_velocity) {
            return velocity_mps.z > 0.0;
        }
        return true;
    }
};

}  // namespace frcsim
