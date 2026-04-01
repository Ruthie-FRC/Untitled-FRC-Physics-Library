#pragma once

#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

enum class BoundaryType {
    kWall,      // Fixed planar barrier
    kPlane,     // Infinite plane constraint
    kBox,       // Axis-aligned or rotated box region
    kCylinder,  // Cylindrical region (e.g., exclusion zone)
};

enum class BoundaryBehavior {
    kRigidBody,        // Treat as a moving or static rigid body with physics interactions
    kStaticConstraint, // Enforce constraint without full physics interactions (faster)
};

struct EnvironmentalBoundary {
    BoundaryType type{BoundaryType::kWall};
    BoundaryBehavior behavior{BoundaryBehavior::kStaticConstraint};

    Vector3 position_m{};
    Quaternion orientation{};

    Vector3 half_extents_m{1.0, 1.0, 1.0};
    double radius_m{1.0};

    // For rigid-body boundaries, physical properties
    double restitution{0.5};
    double friction_coefficient{0.7};

    // Allow external code to tag boundaries
    int user_id{0};
    bool is_active{true};

    const Vector3& normal() const {
        static const Vector3 default_normal(0.0, 0.0, 1.0);
        return default_normal;
    }
};

}  // namespace frcsim
