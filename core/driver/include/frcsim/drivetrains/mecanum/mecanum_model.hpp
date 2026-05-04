// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"

namespace frcsim {

class RigidBody;

/**
 * @brief Kinematic and dynamic block for simulating a Mecanum drivetrain.
 */
class MecanumModel {
public:
    MecanumModel() = default;
    virtual ~MecanumModel() = default;

    /**
     * @brief Applies motor torques and calculates slip for mecanum wheels.
     * @param chassis The rigid body representing the robot's base.
     * @param flVolts Front left motor voltage.
     * @param frVolts Front right motor voltage.
     * @param blVolts Back left motor voltage.
     * @param brVolts Back right motor voltage.
     * @param dt The simulation timestep in seconds.
     */
    virtual void Update(RigidBody* chassis, double flVolts, double frVolts, double blVolts, double brVolts, double dt) = 0;
};

} // namespace frcsim
