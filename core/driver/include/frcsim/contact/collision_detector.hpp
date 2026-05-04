// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"
#include "frcsim/math/quaternion.hpp"
#include "frcsim/collision_shapes/shape.hpp"
#include <vector>

namespace frcsim {

/**
 * @brief Describes a single contact point between two colliding bodies.
 */
struct ContactPoint {
    Vector3 pointOnA;     ///< Contact point in local space of body A.
    Vector3 pointOnB;     ///< Contact point in local space of body B.
    Vector3 worldPoint;   ///< Contact point in world space.
    Vector3 normal;       ///< Collision normal vector pointing from A to B.
    double depth;         ///< Penetration depth (positive means overlap).
};

/**
 * @brief Represents the result of a collision check between two shapes.
 */
struct CollisionManifold {
    std::vector<ContactPoint> contacts;  ///< Generated contact points.
    bool hasCollision = false;           ///< True if shapes are penetrating.
};

/**
 * @brief Pure virtual interface for collision detection algorithms.
 */
class CollisionDetector {
public:
    virtual ~CollisionDetector() = default;

    /**
     * @brief Computes the collision manifold between two generic shapes.
     * @param shapeA Conceptually the first colliding shape.
     * @param posA World position of shapeA.
     * @param rotA World rotation of shapeA.
     * @param shapeB Conceptually the second colliding shape.
     * @param posB World position of shapeB.
     * @param rotB World rotation of shapeB.
     * @return The resulting collision manifold mapping the contact points.
     */
    virtual CollisionManifold DetectCollision(
        const Shape* shapeA, const Vector3& posA, const Quaternion& rotA,
        const Shape* shapeB, const Vector3& posB, const Quaternion& rotB) = 0;
};

} // namespace frcsim
