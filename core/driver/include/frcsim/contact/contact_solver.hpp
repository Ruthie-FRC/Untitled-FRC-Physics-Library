// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/contact/collision_detector.hpp"

namespace frcsim {

class RigidBody; // Forward declaration

/**
 * @brief Solves contact constraints applying impulses to resolve interpenetration and calculate friction.
 */
class ContactSolver {
public:
    virtual ~ContactSolver() = default;

    /**
     * @brief Modifies velocities to resolve contact and penetration.
     * @param bodyA The first rigid body in the manifold.
     * @param bodyB The second rigid body in the manifold.
     * @param manifold The collision points calculated between the two bodies.
     * @param dt The simulation timestep in seconds.
     */
    virtual void SolveContacts(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const CollisionManifold& manifold,
        double dt) = 0;
};

} // namespace frcsim
