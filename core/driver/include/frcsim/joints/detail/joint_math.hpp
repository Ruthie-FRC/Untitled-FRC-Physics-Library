// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <cmath>

#include "frcsim/rigidbody/rigid_body.hpp"

namespace frcsim::detail {

constexpr double kJointEpsilon = 1e-9;
inline const double kPi = std::acos(-1.0);

/**
 * @brief Converts a body-local anchor point into world space.
 * @param body Rigid body that owns the anchor.
 * @param local_anchor Anchor position expressed in the body's local coordinate
 *        frame, in meters.
 * @return World-space anchor point obtained by rotating the local offset by
 *         the body orientation and adding the body position.
 */
inline Vector3 worldAnchor(const RigidBody* body,
                           const Vector3& local_anchor) {
  return body->position() + body->orientation().rotate(local_anchor);
}

/**
 * @brief Clamps a scalar into a closed range.
 * @param value Input value.
 * @param min_value Lower bound.
 * @param max_value Upper bound.
 * @return value constrained to [min_value, max_value].
 */
inline double clampValue(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(value, max_value));
}

/**
 * @brief Applies a positional correction split across two bodies by mass.
 *
 * The correction is distributed using inverse mass so lighter bodies move
 * more than heavier bodies. Kinematic bodies are treated as immovable and
 * therefore receive no positional update.
 *
 * @param body_a First rigid body to correct.
 * @param body_b Second rigid body to correct.
 * @param error World-space position error vector in meters that should be
 *        removed.
 * @param gain Fraction of the error to resolve this step, typically in the
 *        range [0, 1].
 */
inline void applyPositionCorrection(RigidBody* body_a, RigidBody* body_b,
                                    const Vector3& error, double gain) {
  const double inv_a =
      body_a->flags().is_kinematic ? 0.0 : body_a->inverseMass();
  const double inv_b =
      body_b->flags().is_kinematic ? 0.0 : body_b->inverseMass();
  const double total_inv = inv_a + inv_b;
  if (total_inv <= kJointEpsilon)
    return;

  const Vector3 correction_a = error * (gain * (inv_a / total_inv));
  const Vector3 correction_b = error * (gain * (inv_b / total_inv));

  if (!body_a->flags().is_kinematic) {
    body_a->setPosition(body_a->position() + correction_a);
  }
  if (!body_b->flags().is_kinematic) {
    body_b->setPosition(body_b->position() - correction_b);
  }
}

/**
 * @brief Applies a velocity-space correction split across two bodies by mass.
 *
 * This is the velocity analogue of applyPositionCorrection(): the supplied
 * relative velocity impulse is divided by inverse mass so the lighter body
 * receives the larger change. Kinematic bodies are left unchanged.
 *
 * @param body_a First rigid body to correct.
 * @param body_b Second rigid body to correct.
 * @param relative_velocity World-space relative velocity vector in meters per
 *        second to reduce.
 * @param gain Fraction of the velocity error to resolve this step.
 */
inline void applyVelocityCorrection(RigidBody* body_a, RigidBody* body_b,
                                    const Vector3& relative_velocity,
                                    double gain) {
  const double inv_a =
      body_a->flags().is_kinematic ? 0.0 : body_a->inverseMass();
  const double inv_b =
      body_b->flags().is_kinematic ? 0.0 : body_b->inverseMass();
  const double total_inv = inv_a + inv_b;
  if (total_inv <= kJointEpsilon)
    return;

  const Vector3 correction_a = relative_velocity * (gain * (inv_a / total_inv));
  const Vector3 correction_b = relative_velocity * (gain * (inv_b / total_inv));

  if (!body_a->flags().is_kinematic) {
    body_a->setLinearVelocity(body_a->linearVelocity() + correction_a);
  }
  if (!body_b->flags().is_kinematic) {
    body_b->setLinearVelocity(body_b->linearVelocity() - correction_b);
  }
}

/**
 * @brief Rotates a local axis into world space and falls back if needed.
 * @param body Body whose orientation defines the local axis transform.
 * @param local_axis Axis in the body's local coordinate frame, treated as a
 *        direction vector rather than a position and therefore unitless.
 * @param fallback_axis World-space fallback direction, also unitless.
 * @return Normalized world-space axis, or fallback_axis if the rotated axis
 *         collapses to zero length.
 */
inline Vector3 worldAxisOrFallback(const RigidBody* body,
                                   const Vector3& local_axis,
                                   const Vector3& fallback_axis) {
  Vector3 axis_world = body->orientation().rotate(local_axis).normalized();
  return axis_world.isZero() ? fallback_axis : axis_world;
}

/**
 * @brief Computes the signed twist between two orientations about an axis.
 *
 * The helper compares qb against qa, extracts the shortest relative rotation,
 * and projects that rotation onto axis_world using the right-hand rule. The
 * return value is a signed angle in radians: its magnitude is the amount of
 * twist, and its sign indicates whether the target orientation rotates in the
 * positive or negative direction about the supplied axis.
 *
 * The intent is to match the coordinate conventions used throughout WPILib:
 * a right-handed field frame with +X downfield, +Y left/right across the
 * field depending on alliance perspective, and +Z upward. That convention is
 * documented here:
 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
 *
 * Keep qa, qb, and axis_world in the same world frame when calling this
 * helper. If axis_world is degenerate, the normalized axis collapses toward
 * zero and the returned signed twist likewise tends toward zero.
 *
 * @param qa Reference orientation.
 * @param qb Target orientation to compare against qa.
 * @param axis_world World-space rotation axis, expressed as a unit direction
 *        vector when possible.
 * @return Signed twist angle in radians about axis_world.
 */
inline double signedTwistAngleRad(const Quaternion& qa, const Quaternion& qb,
                                  const Vector3& axis_world) {
  Quaternion q_rel = qb * qa.inverse();
  q_rel.normalizeIfNeeded();

  Vector3 axis_err;
  double angle = 0.0;
  q_rel.toAxisAngle(axis_err, angle);
  if (angle > kPi) {
    angle = 2.0 * kPi - angle;
    axis_err = -axis_err;
  }
  return angle * axis_err.dot(axis_world.normalized());
}

}  // namespace frcsim::detail
