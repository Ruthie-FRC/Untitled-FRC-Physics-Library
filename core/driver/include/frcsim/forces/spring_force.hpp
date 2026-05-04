// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/forces/force_generator.hpp"
#include "frcsim/math/vector.hpp"

namespace frcsim {

class RigidBody;

/**
 * @brief Applies proportional restorative force based on Hooke's Law.
 */
class SpringForce : public ForceGenerator {
public:
    /**
     * @brief Constructs a spring force generator.
     * @param otherBody The body this spring is attached to at the other end.
     * @param localConnectionPoint Local position of the connection point on the object.
     * @param otherConnectionPoint Local position of the connection point on the other body.
     * @param springConstant The stiffness (k).
     * @param restLength The natural resting length.
     */
    SpringForce(RigidBody* otherBody, const Vector3& localConnectionPoint, const Vector3& otherConnectionPoint, double springConstant, double restLength);
    
    virtual ~SpringForce() = default;

    /**
     * @brief Applies spring force to registered objects.
     * @param duration Timestep length.
     */
    void UpdateForce(double duration) override;

private:
    RigidBody* m_otherBody;
    Vector3 m_localConnectionPoint;
    Vector3 m_otherConnectionPoint;
    double m_springConstant;
    double m_restLength;
};

inline SpringForce::SpringForce(RigidBody* otherBody, const Vector3& localConnectionPoint, const Vector3& otherConnectionPoint, double springConstant, double restLength)
    : m_otherBody(otherBody), m_localConnectionPoint(localConnectionPoint), m_otherConnectionPoint(otherConnectionPoint), m_springConstant(springConstant), m_restLength(restLength) {}

inline void SpringForce::UpdateForce(double duration) {
    // Implementation placeholder
}

} // namespace frcsim
