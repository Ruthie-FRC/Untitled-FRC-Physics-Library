// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/collision_shapes/shape.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

/**
 * @brief Represents a rectangular prism collision shape.
 */
class Box : public Shape {
public:
    /**
     * @brief Constructs a new Box centered at the local origin.
     * @param halfExtents A Vector3 where x, y, and z are half the width, height, and depth respectively.
     */
    explicit Box(const Vector3& halfExtents);

    virtual ~Box() = default;

    ShapeType GetType() const override;
    double CalculateVolume() const override;
    Vector3 CalculateLocalInertia(double mass) const override;

    /**
     * @brief Gets the half extents of the box.
     * @return The dimensions of the box from the center to its edges.
     */
    const Vector3& GetHalfExtents() const;

private:
    Vector3 m_halfExtents;
};

inline Box::Box(const Vector3& halfExtents) : m_halfExtents(halfExtents) {}

inline ShapeType Box::GetType() const { return ShapeType::BOX; }

inline double Box::CalculateVolume() const {
    return 8.0 * m_halfExtents.x * m_halfExtents.y * m_halfExtents.z;
}

inline Vector3 Box::CalculateLocalInertia(double mass) const {
    double lx2 = 4.0 * m_halfExtents.x * m_halfExtents.x;
    double ly2 = 4.0 * m_halfExtents.y * m_halfExtents.y;
    double lz2 = 4.0 * m_halfExtents.z * m_halfExtents.z;
    double factor = mass / 12.0;
    return Vector3(factor * (ly2 + lz2), factor * (lx2 + lz2), factor * (lx2 + ly2));
}

inline const Vector3& Box::GetHalfExtents() const { return m_halfExtents; }

} // namespace frcsim
