// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/collision_shapes/shape.hpp"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>

namespace frcsim {

/**
 * @brief Represents a perfect sphere collision shape.
 */
class Sphere : public Shape {
public:
    /**
     * @brief Constructs a new Sphere centered at the local origin.
     * @param radius The radius of the sphere in meters.
     */
    explicit Sphere(double radius);

    virtual ~Sphere() = default;

    ShapeType GetType() const override;
    double CalculateVolume() const override;
    Vector3 CalculateLocalInertia(double mass) const override;

    /**
     * @brief Gets the radius of the sphere.
     * @return The radius in meters.
     */
    double GetRadius() const;

private:
    double m_radius;
};

inline Sphere::Sphere(double radius) : m_radius(radius) {}

inline ShapeType Sphere::GetType() const { return ShapeType::SPHERE; }

inline double Sphere::CalculateVolume() const {
    return (4.0 / 3.0) * M_PI * m_radius * m_radius * m_radius;
}

inline Vector3 Sphere::CalculateLocalInertia(double mass) const {
    double i = 0.4 * mass * m_radius * m_radius;
    return Vector3(i, i, i);
}

inline double Sphere::GetRadius() const { return m_radius; }

} // namespace frcsim
