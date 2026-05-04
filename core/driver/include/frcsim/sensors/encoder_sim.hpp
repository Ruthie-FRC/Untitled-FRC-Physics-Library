// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/math/vector.hpp"

namespace frcsim {

/**
 * @brief Simulates an absolute or relative encoder mapping to mechanical state.
 */
class EncoderSim {
public:
    EncoderSim() = default;
    virtual ~EncoderSim() = default;

    /**
     * @brief Returns the simulated raw position or tick count.
     * @return Simulated position.
     */
    virtual double GetPosition() const = 0;

    /**
     * @brief Returns the simulated velocity/rate.
     * @return Simulated rate.
     */
    virtual double GetRate() const = 0;
};

} // namespace frcsim
