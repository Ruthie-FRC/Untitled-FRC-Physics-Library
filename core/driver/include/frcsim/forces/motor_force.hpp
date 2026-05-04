// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/forces/force_generator.hpp"

namespace frcsim {

/**
 * @brief Applies rotational or linear force simulating an electric motor.
 */
class MotorForce : public ForceGenerator {
public:
    MotorForce() = default;
    virtual ~MotorForce() = default;

    /**
     * @brief Applies simulated motor force to the registered object.
     * @param duration Timestep length.
     */
    void UpdateForce(double duration) override;

    /**
     * @brief Sets the input voltage or current.
     * @param voltage The input level.
     */
    void SetInput(double voltage);

private:
    double m_inputVoltage = 0.0;
};

inline void MotorForce::UpdateForce(double duration) {
    // Basic implementation override
}

inline void MotorForce::SetInput(double voltage) {
    m_inputVoltage = voltage;
}

} // namespace frcsim
