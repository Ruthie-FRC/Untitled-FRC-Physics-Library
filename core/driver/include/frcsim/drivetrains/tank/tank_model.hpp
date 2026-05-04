// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"

namespace frcsim {

class RigidBody;

/**
 * @brief Kinematic and dynamic model for simulating a differential/tank drive.
 */
class TankModel {
public:
    TankModel() = default;
    virtual ~TankModel() = default;

    /**
     * @brief Updates the chassis dynamics using left and right wheel speeds/torques.
     * @param chassis The rigid body representing the robot.
     * @param leftVolts Left side motor voltage.
     * @param rightVolts Right side motor voltage.
     * @param dt Timestep in seconds.
     */
    virtual void Update(RigidBody* chassis, double leftVolts, double rightVolts, double dt) = 0;
};

} // namespace frcsim
