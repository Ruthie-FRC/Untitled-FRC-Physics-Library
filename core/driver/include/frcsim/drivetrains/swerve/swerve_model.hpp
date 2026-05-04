// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"
#include <vector>

namespace frcsim {

class RigidBody;

/**
 * @brief Represents the state of a single swerve module.
 */
struct SwerveModuleState {
    double speed; ///< Speed in m/s.
    double angle; ///< Angle in radians.
};

/**
 * @brief Kinematic and dynamic block for simulating a Swerve drivetrain.
 */
class SwerveModel {
public:
    SwerveModel() = default;
    virtual ~SwerveModel() = default;

    /**
     * @brief Modifies chassis velocities and apply friction based on swerve module states.
     * @param chassis The rigid body representing the robot's base.
     * @param states The array of 4 swerve module states (FL, FR, BL, BR).
     * @param dt Simulation timestep in seconds.
     */
    virtual void Update(RigidBody* chassis, const std::vector<SwerveModuleState>& states, double dt) = 0;
};

} // namespace frcsim
