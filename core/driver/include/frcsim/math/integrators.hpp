// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "vector.hpp"
#include "quaternion.hpp"

namespace frcsim {

/**
 * @brief Time-integration helpers for translational and rotational rigid-body
 * dynamics.
 *
 * This utility exposes explicit Euler, semi-implicit (symplectic) Euler, and
 * midpoint RK2 variants for linear motion, plus quaternion-based angular
 * integration using q_dot = 0.5 * omega_quat * q.
 *
 * References:
 * https://en.wikipedia.org/wiki/Semi-implicit_Euler_method
 * https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
 * https://en.wikipedia.org/wiki/Quaternion#Using_quaternions_as_rotations
 */
struct Integrator {
  // Semi-Implicit Euler (Linear)
  // velocity += acceleration * dt
  // position += velocity * dt
  // Stable and standard for real-time physics engines.

  static inline void integrateLinear(Vector3& position, Vector3& velocity,
                                     const Vector3& acceleration,
                                     double dt) noexcept {
    velocity += acceleration * dt;
    position += velocity * dt;
  }

  // Explicit Euler (optional)
  // Useful for debugging or predictor steps

  static inline void integrateLinearExplicit(Vector3& position,
                                             Vector3& velocity,
                                             const Vector3& acceleration,
                                             double dt) noexcept {
    position += velocity * dt;
    velocity += acceleration * dt;
  }

  // Angular Integration
  // Quaternion derivative:
  //   q_dot = 0.5 * omega_quat * q
  // where omega_quat = (0, wx, wy, wz)
  static inline void integrateAngular(Quaternion& orientation,
                                      const Vector3& angularVelocity,
                                      double dt) noexcept {
    Quaternion omegaQuat(0.0, angularVelocity.x, angularVelocity.y,
                         angularVelocity.z);
    Quaternion dq = omegaQuat * orientation * 0.5;
    orientation = orientation + dq * dt;
    orientation.normalizeIfNeeded();
  }

  // Angular Velocity Integration: omega += alpha * dt

  static inline void integrateAngularVelocity(
      Vector3& angularVelocity, const Vector3& angularAcceleration,
      double dt) noexcept {
    angularVelocity += angularAcceleration * dt;
  }

  // RK2 Integrator (optional)
  // Midpoint integration, useful for projectiles or high-speed mechanisms.

  static inline void integrateLinearRK2(Vector3& position, Vector3& velocity,
                                        const Vector3& acceleration,
                                        double dt) noexcept {
    Vector3 halfVelocity = velocity + acceleration * (0.5 * dt);

    position += halfVelocity * dt;

    velocity += acceleration * dt;
  }
};

}  // namespace frcsim
