// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"
#include "frcsim/math/quaternion.hpp"
#include <vector>

namespace frcsim {

/**
 * @brief Simulates a camera/vision coprocessor (e.g. PhotonVision or Limelight).
 */
class VisionSim {
public:
    VisionSim() = default;
    virtual ~VisionSim() = default;

    /**
     * @brief Updates the vision simulation based on current robot and field poses.
     * @param cameraPosition World position of the camera.
     * @param cameraRotation World orientation of the camera.
     * @param dt Timestep duration.
     */
    virtual void Update(const Vector3& cameraPosition, const Quaternion& cameraRotation, double dt) = 0;

    /**
     * @brief Returns a boolean indicating if targets are visible in the fov.
     */
    virtual bool HasTargets() const = 0;
};

} // namespace frcsim
