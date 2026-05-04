// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

namespace frcsim {

/**
 * @brief Simulates a linear elevator mechanism.
 */
class Elevator {
public:
    Elevator() = default;
    virtual ~Elevator() = default;

    /**
     * @brief Updates the elevator state based on applied voltage and physics.
     * @param voltage The input voltage.
     * @param dt The timestep in seconds.
     */
    virtual void Update(double voltage, double dt) = 0;

    /**
     * @brief Gets the current position of the elevator.
     * @return The position in meters.
     */
    virtual double GetPosition() const = 0;

    /**
     * @brief Gets the current velocity of the elevator.
     * @return The velocity in meters per second.
     */
    virtual double GetVelocity() const = 0;
};

} // namespace frcsim
