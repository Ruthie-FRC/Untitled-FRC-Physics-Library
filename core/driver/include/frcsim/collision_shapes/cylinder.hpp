// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/collision_shapes/shape.hpp"
#include "frcsim/math/vector.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>

namespace frcsim {

/**
 * @brief Represents a cylinder collision shape aligned with the Y axis.
 */
class Cylinder : public Shape {
public:
    /**
     * @brief Constructs a new Cylinder.
     * @param radius The radius of the cylinder base in meters.
     * @param halfHeight Half the height of the cylinder along the Y axis in meters.
     */
    Cylinder(double radius, double halfHeight);

    virtual ~Cylinder() = default;

    ShapeType GetType() const override;
    double CalculateVolume() const override;
    Vector3 CalculateLocalInertia(double mass) const override;

    /**
     * @brief Gets the radius of the cylinder.
     * @return The radius in meters.
     */
    double GetRadius() const;

    /**
     * @brief Gets the half-height of the cylinder.
     * @return Half the total height (Y-axis extents) in meters.
     */
    double GetHalfHeight() const;

private:
    double m_radius;
    double m_halfHeight;
};

inline Cylinder::Cylinder(double radius, double halfHeight) 
    : m_radius(radius), m_halfHeight(halfHeight) {}

inline ShapeType Cylinder::GetType() const { return ShapeType::CYLINDER; }

inline double Cylinder::CalculateVolume() const {
    return M_PI * m_radius * m_radius * 2.0 * m_halfHeight;
}

inline Vector3 Cylinder::CalculateLocalInertia(double mass) const {
    double r2 = m_radius * m_radius;
    double h2 = 4.0 * m_halfHeight * m_halfHeight;
    double ixz = (1.0 / 12.0) * mass * (3.0 * r2 + h2);
    double iy = 0.5 * mass * r2;
    return Vector3(ixz, iy, ixz);
}

inline double Cylinder::GetRadius() const { return m_radius; }
inline double Cylinder::GetHalfHeight() const { return m_halfHeight; }

} // namespace frcsim
