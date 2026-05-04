// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/quaternion.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

/**
 * @brief Simulates an IMU/Gyroscope, incorporating drift and noise models.
 */
class GyroSim {
public:
    GyroSim() = default;
    virtual ~GyroSim() = default;

    /**
     * @brief Gets simulated angular position (yaw, pitch, roll).
     * @return Simulated Euler angles or rotation.
     */
    virtual Quaternion GetOrientation() const = 0;

    /**
     * @brief Gets rotational velocity vector.
     * @return Radians per second for x, y, z axes.
     */
    virtual Vector3 GetAngularVelocity() const = 0;
};

} // namespace frcsim
