// Copyright (c) RenSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#include "frcsim/joints/fixed_joint.hpp"
#include "frcsim/joints/prismatic_joint.hpp"
#include "frcsim/joints/revolute_joint.hpp"
#include "frcsim/rigidbody/rigid_body.hpp"

#include <algorithm>
#include <cmath>

namespace {

using frcsim::Quaternion;
using frcsim::RigidBody;
using frcsim::Vector3;

constexpr double kEps = 1e-9;

Vector3 worldAnchor(const RigidBody* body, const Vector3& local_anchor) {
  return body->position() + body->orientation().rotate(local_anchor);
}

double clampValue(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(value, max_value));
}

void applyPositionCorrection(RigidBody* body_a, RigidBody* body_b,
                             const Vector3& error, double gain) {
  const double inv_a =
      body_a->flags().is_kinematic ? 0.0 : body_a->inverseMass();
  const double inv_b =
      body_b->flags().is_kinematic ? 0.0 : body_b->inverseMass();
  const double total_inv = inv_a + inv_b;
  if (total_inv <= kEps)
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

void applyVelocityCorrection(RigidBody* body_a, RigidBody* body_b,
                             const Vector3& relative_velocity, double gain) {
  const double inv_a =
      body_a->flags().is_kinematic ? 0.0 : body_a->inverseMass();
  const double inv_b =
      body_b->flags().is_kinematic ? 0.0 : body_b->inverseMass();
  const double total_inv = inv_a + inv_b;
  if (total_inv <= kEps)
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

double signedTwistAngleRad(const Quaternion& qa, const Quaternion& qb,
                           const Vector3& axis_world) {
  Quaternion q_rel = qb * qa.inverse();
  q_rel.normalizeIfNeeded();

  Vector3 axis_err;
  double angle = 0.0;
  q_rel.toAxisAngle(axis_err, angle);
  if (angle > 3.14159265358979323846) {
    angle = 2.0 * 3.14159265358979323846 - angle;
    axis_err = -axis_err;
  }
  return angle * axis_err.dot(axis_world.normalized());
}

}  // namespace

namespace frcsim {

void FixedJoint::solveConstraint(double /*dt_s*/, int /*iterations*/) {
  if (!is_enabled_ || !body_a_ || !body_b_)
    return;

  const Vector3 world_a = worldAnchor(body_a_, anchor_a_);
  const Vector3 world_b = worldAnchor(body_b_, anchor_b_);
  const Vector3 positional_error = world_b - world_a;
  applyPositionCorrection(body_a_, body_b_, positional_error, 0.8);

  const Vector3 relative_linear_velocity =
      body_b_->linearVelocity() - body_a_->linearVelocity();
  applyVelocityCorrection(body_a_, body_b_, relative_linear_velocity, 0.65);

  const double inv_a =
      body_a_->flags().is_kinematic ? 0.0 : body_a_->inverseMass();
  const double inv_b =
      body_b_->flags().is_kinematic ? 0.0 : body_b_->inverseMass();
  const double total_inv = inv_a + inv_b;
  if (total_inv <= kEps)
    return;

  const Vector3 relative_angular_velocity =
      body_b_->angularVelocity() - body_a_->angularVelocity();
  const Vector3 angular_correction_a =
      relative_angular_velocity * (0.6 * (inv_a / total_inv));
  const Vector3 angular_correction_b =
      relative_angular_velocity * (0.6 * (inv_b / total_inv));
  if (!body_a_->flags().is_kinematic) {
    body_a_->setAngularVelocity(body_a_->angularVelocity() +
                                angular_correction_a);
  }
  if (!body_b_->flags().is_kinematic) {
    body_b_->setAngularVelocity(body_b_->angularVelocity() -
                                angular_correction_b);
  }

  Quaternion qa = body_a_->orientation();
  Quaternion qb = body_b_->orientation();
  Quaternion q_rel = qb * qa.inverse();
  q_rel.normalizeIfNeeded();
  Vector3 axis;
  double angle = 0.0;
  q_rel.toAxisAngle(axis, angle);
  if (angle > 3.14159265358979323846) {
    angle = 2.0 * 3.14159265358979323846 - angle;
    axis = -axis;
  }

  if (angle > 1e-6 && !axis.isZero()) {
    const double correction = clampValue(0.4 * angle, 0.0, 0.25);
    if (!body_a_->flags().is_kinematic) {
      const Quaternion q_corr_a =
          Quaternion::fromAxisAngle(axis, correction * (inv_a / total_inv));
      qa = q_corr_a * qa;
      qa.normalizeIfNeeded();
      body_a_->setOrientation(qa);
    }
    if (!body_b_->flags().is_kinematic) {
      const Quaternion q_corr_b =
          Quaternion::fromAxisAngle(axis, -correction * (inv_b / total_inv));
      qb = q_corr_b * qb;
      qb.normalizeIfNeeded();
      body_b_->setOrientation(qb);
    }
  }
}

double FixedJoint::constraintError() const {
  if (!body_a_ || !body_b_)
    return 0.0;

  const Vector3 world_a = worldAnchor(body_a_, anchor_a_);
  const Vector3 world_b = worldAnchor(body_b_, anchor_b_);
  const double positional_error = (world_b - world_a).norm();

  Quaternion q_rel = body_b_->orientation() * body_a_->orientation().inverse();
  q_rel.normalizeIfNeeded();
  Vector3 axis;
  double angle = 0.0;
  q_rel.toAxisAngle(axis, angle);
  if (angle > 3.14159265358979323846) {
    angle = 2.0 * 3.14159265358979323846 - angle;
  }

  return positional_error + angle;
}

void RevoluteJoint::solveConstraint(double dt_s, int iterations) {
  if (!is_enabled_ || !body_a_ || !body_b_)
    return;

  const Vector3 axis_world =
      body_a_->orientation().rotate(axis_local_).normalized().isZero()
          ? Vector3::unitZ()
          : body_a_->orientation().rotate(axis_local_).normalized();

  for (int i = 0; i < std::max(1, iterations); ++i) {
    const Vector3 world_a = worldAnchor(body_a_, anchor_a_);
    const Vector3 world_b = worldAnchor(body_b_, anchor_b_);
    const Vector3 positional_error = world_b - world_a;
    applyPositionCorrection(body_a_, body_b_, positional_error, 0.7);

    const Vector3 rel_ang =
        body_b_->angularVelocity() - body_a_->angularVelocity();
    const Vector3 orthogonal_rel_ang =
        rel_ang - axis_world * rel_ang.dot(axis_world);

    const double inv_a =
        body_a_->flags().is_kinematic ? 0.0 : body_a_->inverseMass();
    const double inv_b =
        body_b_->flags().is_kinematic ? 0.0 : body_b_->inverseMass();
    const double total_inv = inv_a + inv_b;
    if (total_inv > kEps) {
      if (!body_a_->flags().is_kinematic) {
        body_a_->setAngularVelocity(body_a_->angularVelocity() +
                                    orthogonal_rel_ang *
                                        (0.6 * inv_a / total_inv));
      }
      if (!body_b_->flags().is_kinematic) {
        body_b_->setAngularVelocity(body_b_->angularVelocity() -
                                    orthogonal_rel_ang *
                                        (0.6 * inv_b / total_inv));
      }

      if (has_motor_ && dt_s > 0.0) {
        const double current_rel = rel_ang.dot(axis_world);
        const double target_delta = motor_target_velocity_radps_ - current_rel;
        const double max_step = std::max(0.0, motor_max_torque_nm_) * dt_s;
        const double applied_delta =
            clampValue(target_delta, -max_step, max_step);
        if (!body_a_->flags().is_kinematic) {
          body_a_->setAngularVelocity(body_a_->angularVelocity() -
                                      axis_world *
                                          (applied_delta * inv_a / total_inv));
        }
        if (!body_b_->flags().is_kinematic) {
          body_b_->setAngularVelocity(body_b_->angularVelocity() +
                                      axis_world *
                                          (applied_delta * inv_b / total_inv));
        }
      }

      if (has_limits_ && dt_s > 0.0) {
        const double rel_angle = signedTwistAngleRad(
            body_a_->orientation(), body_b_->orientation(), axis_world);
        double limit_violation = 0.0;
        if (rel_angle < min_angle_rad_) {
          limit_violation = min_angle_rad_ - rel_angle;
        } else if (rel_angle > max_angle_rad_) {
          limit_violation = max_angle_rad_ - rel_angle;
        }

        if (std::abs(limit_violation) > 1e-8) {
          const double correction_speed =
              clampValue(limit_violation / dt_s, -5.0, 5.0);
          if (!body_a_->flags().is_kinematic) {
            body_a_->setAngularVelocity(
                body_a_->angularVelocity() -
                axis_world * (correction_speed * inv_a / total_inv));
          }
          if (!body_b_->flags().is_kinematic) {
            body_b_->setAngularVelocity(
                body_b_->angularVelocity() +
                axis_world * (correction_speed * inv_b / total_inv));
          }
        }
      }
    }
  }
}

double RevoluteJoint::constraintError() const {
  if (!body_a_ || !body_b_)
    return 0.0;

  const Vector3 axis_world =
      body_a_->orientation().rotate(axis_local_).normalized().isZero()
          ? Vector3::unitZ()
          : body_a_->orientation().rotate(axis_local_).normalized();
  const Vector3 world_a = worldAnchor(body_a_, anchor_a_);
  const Vector3 world_b = worldAnchor(body_b_, anchor_b_);
  const double anchor_error = (world_b - world_a).norm();

  const Vector3 rel_ang =
      body_b_->angularVelocity() - body_a_->angularVelocity();
  const Vector3 orthogonal_rel_ang =
      rel_ang - axis_world * rel_ang.dot(axis_world);
  double error = anchor_error + orthogonal_rel_ang.norm();

  if (has_limits_) {
    const double rel_angle = signedTwistAngleRad(
        body_a_->orientation(), body_b_->orientation(), axis_world);
    if (rel_angle < min_angle_rad_) {
      error += (min_angle_rad_ - rel_angle);
    } else if (rel_angle > max_angle_rad_) {
      error += (rel_angle - max_angle_rad_);
    }
  }

  return error;
}

void PrismaticJoint::solveConstraint(double dt_s, int iterations) {
  if (!is_enabled_ || !body_a_ || !body_b_)
    return;

  const Vector3 axis_world =
      body_a_->orientation().rotate(axis_local_).normalized().isZero()
          ? Vector3::unitX()
          : body_a_->orientation().rotate(axis_local_).normalized();

  for (int i = 0; i < std::max(1, iterations); ++i) {
    const Vector3 world_a = worldAnchor(body_a_, anchor_a_);
    const Vector3 world_b = worldAnchor(body_b_, anchor_b_);
    const Vector3 delta = world_b - world_a;

    const double axial_disp = delta.dot(axis_world);
    const Vector3 axial_component = axis_world * axial_disp;
    const Vector3 lateral_component = delta - axial_component;
    applyPositionCorrection(body_a_, body_b_, lateral_component, 0.8);

    const double inv_a =
        body_a_->flags().is_kinematic ? 0.0 : body_a_->inverseMass();
    const double inv_b =
        body_b_->flags().is_kinematic ? 0.0 : body_b_->inverseMass();
    const double total_inv = inv_a + inv_b;
    if (total_inv <= kEps)
      continue;

    if (has_limits_) {
      double clamped_disp = axial_disp;
      if (axial_disp < min_displacement_m_) {
        clamped_disp = min_displacement_m_;
      } else if (axial_disp > max_displacement_m_) {
        clamped_disp = max_displacement_m_;
      }

      const double violation = axial_disp - clamped_disp;
      if (std::abs(violation) > 1e-8) {
        const Vector3 limit_correction = axis_world * violation;
        applyPositionCorrection(body_a_, body_b_, limit_correction, 0.75);
      }
    }

    const Vector3 rel_v = body_b_->linearVelocity() - body_a_->linearVelocity();
    const double rel_v_axis = rel_v.dot(axis_world);
    const Vector3 rel_v_lateral = rel_v - axis_world * rel_v_axis;
    applyVelocityCorrection(body_a_, body_b_, rel_v_lateral, 0.7);

    if (has_motor_ && dt_s > 0.0) {
      const double delta_v = motor_target_velocity_mps_ - rel_v_axis;
      const double max_step = std::max(0.0, motor_max_force_n_) * dt_s;
      const double applied_delta = clampValue(delta_v, -max_step, max_step);
      if (!body_a_->flags().is_kinematic) {
        body_a_->setLinearVelocity(body_a_->linearVelocity() -
                                   axis_world *
                                       (applied_delta * inv_a / total_inv));
      }
      if (!body_b_->flags().is_kinematic) {
        body_b_->setLinearVelocity(body_b_->linearVelocity() +
                                   axis_world *
                                       (applied_delta * inv_b / total_inv));
      }
    }
  }
}

double PrismaticJoint::constraintError() const {
  if (!body_a_ || !body_b_)
    return 0.0;

  const Vector3 axis_world =
      body_a_->orientation().rotate(axis_local_).normalized().isZero()
          ? Vector3::unitX()
          : body_a_->orientation().rotate(axis_local_).normalized();

  const Vector3 world_a = worldAnchor(body_a_, anchor_a_);
  const Vector3 world_b = worldAnchor(body_b_, anchor_b_);
  const Vector3 delta = world_b - world_a;
  const double axial_disp = delta.dot(axis_world);
  const Vector3 lateral_component = delta - axis_world * axial_disp;

  double error = lateral_component.norm();
  if (has_limits_) {
    if (axial_disp < min_displacement_m_) {
      error += min_displacement_m_ - axial_disp;
    } else if (axial_disp > max_displacement_m_) {
      error += axial_disp - max_displacement_m_;
    }
  }

  return error;
}

}  // namespace frcsim
