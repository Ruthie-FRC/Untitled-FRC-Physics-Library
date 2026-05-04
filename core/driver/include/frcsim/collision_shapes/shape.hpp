// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"

namespace frcsim {

/**
 * @brief Enum identifying the specific types of collision shapes available.
 */
enum class ShapeType {
    BOX,
    SPHERE,
    CYLINDER,
    MESH,
    UNKNOWN
};

/**
 * @brief Base class for all collision geometries in the physics engine.
 *
 * Defines the common interface for collision detection, volume calculations,
 * and inertial property computation.
 */
class Shape {
public:
    virtual ~Shape() = default;

    /**
     * @brief Gets the exact type of this shape.
     * @return The ShapeType enum value corresponding to the derived class.
     */
    virtual ShapeType GetType() const = 0;

    /**
     * @brief Calculates the volume of the specific shape.
     * @return The geometric volume in cubic meters.
     */
    virtual double CalculateVolume() const = 0;

    /**
     * @brief Computes the diagonal of the local inertia tensor for this shape given a mass.
     * @param mass The mass of the rigid body attached to this shape in kg.
     * @return A Vector3 representing the moments of inertia (Ixx, Iyy, Izz).
     */
    virtual Vector3 CalculateLocalInertia(double mass) const = 0;
};

} // namespace frcsim
