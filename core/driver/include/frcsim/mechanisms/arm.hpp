// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

namespace frcsim {

/**
 * @brief Simulates a rotational arm mechanism.
 */
class Arm {
public:
    Arm() = default;
    virtual ~Arm() = default;

    /**
     * @brief Updates the arm state based on applied voltage and physics.
     * @param voltage The input voltage.
     * @param dt The timestep in seconds.
     */
    virtual void Update(double voltage, double dt) = 0;

    /**
     * @brief Gets the current angle of the arm.
     * @return The angle in radians.
     */
    virtual double GetAngle() const = 0;

    /**
     * @brief Gets the current angular velocity of the arm.
     * @return The velocity in radians per second.
     */
    virtual double GetVelocity() const = 0;
};

} // namespace frcsim
