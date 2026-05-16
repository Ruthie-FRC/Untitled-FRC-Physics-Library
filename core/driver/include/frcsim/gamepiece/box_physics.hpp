// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "frcsim/math/vector.hpp"

namespace frcsim {

/**
 * @brief 3D rigid-body style rectangular-prism simulator with drag and ground
 * contact.
 *
 * This is a separate hitbox type from @ref BallPhysicsSim3D. It models a
 * centered box with full length/width/height dimensions and keeps the shape
 * axis-aligned, which is sufficient for centered hitbox use cases.
 */
class BoxPhysicsSim3D {
 public:
  /** @brief Runtime physics environment parameters. */
  struct Config {
    /** Constant gravity vector applied each integration substep. */
    Vector3 gravity_mps2{0.0, 0.0, -9.81};
    /** Multiplier applied to gravity_mps2 after sanitization. */
    double effective_gravity_scale{1.0};
    /** Air density used by drag force computation. */
    double air_density_kgpm3{1.225};
    /** Additional scale factor applied to drag force. */
    double drag_scale{1.0};
    /** World Z height for the flat ground plane. */
    double ground_height_m{0.0};
    /** Exponential-style planar speed decay while on ground, per second. */
    double rolling_friction_per_s{1.2};
    /** Minimum downward impact speed that triggers a bounce. */
    double min_bounce_speed_mps{0.1};
    /** Maximum internal integration substep size. */
    double max_substep_s{0.01};
  };

  /** @brief Physical parameters for the box hitbox. */
  struct BoxProperties {
    /** Box mass in kilograms. */
    double mass_kg{0.27};
    /** Full box dimensions in meters, ordered as length/width/height. */
    Vector3 dimensions_m{0.24, 0.24, 0.24};
    /** Dimensionless drag coefficient. */
    double drag_coefficient{0.47};
    /** Reference frontal area used for drag in square meters. */
    double reference_area_m2{0.0};
    /** Coefficient of restitution for ground impacts in [0, 1]. */
    double restitution{0.45};
  };

  /** @brief Dynamic state advanced by step(). */
  struct BoxState {
    /** Box center position in world coordinates. */
    Vector3 position_m{};
    /** Box linear velocity in world frame. */
    Vector3 velocity_mps{};
  };

  BoxPhysicsSim3D() = default;

  explicit BoxPhysicsSim3D(const Config& config)
      : config_(sanitizeConfig(config)) {}

  BoxPhysicsSim3D(const Config& config, const BoxProperties& box_properties)
      : config_(sanitizeConfig(config)),
        box_properties_(sanitizeBoxProperties(box_properties)) {}

  /** @brief Returns the active, sanitized configuration. */
  const Config& config() const { return config_; }

  /** @brief Replaces configuration after value sanitization and clamping. */
  void setConfig(const Config& config) { config_ = sanitizeConfig(config); }

  /** @brief Returns the active, sanitized box properties. */
  const BoxProperties& boxProperties() const { return box_properties_; }

  /** @brief Replaces box properties after sanitization. */
  void setBoxProperties(const BoxProperties& props) {
    box_properties_ = sanitizeBoxProperties(props);
  }

  /** @brief Returns the current box state. */
  const BoxState& state() const { return state_; }

  /** @brief Replaces state and sanitizes non-finite values. */
  void setState(const BoxState& state) {
    state_ = state;
    sanitizeState(state_);
  }

  /**
   * @brief Advances simulation by dt seconds.
   * @param dt_s Simulation timestep in seconds.
   */
  void step(double dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      return;
    }

    sanitizeState(state_);

    const double max_substep_s = std::clamp(config_.max_substep_s, 1e-4, 0.05);
    double remaining_s = dt_s;
    while (remaining_s > 0.0) {
      const double substep_s = std::min(remaining_s, max_substep_s);
      const Vector3 accel0_mps2 = computeAcceleration(state_.velocity_mps);
      const Vector3 mid_velocity_mps =
          state_.velocity_mps + accel0_mps2 * (0.5 * substep_s);
      const Vector3 accel_mid_mps2 = computeAcceleration(mid_velocity_mps);

      state_.velocity_mps += accel_mid_mps2 * substep_s;
      state_.position_m += mid_velocity_mps * substep_s;
      sanitizeState(state_);

      remaining_s -= substep_s;
    }

    resolveGroundContact(dt_s);
    sanitizeState(state_);
  }

 private:
  /** @brief Returns true when all vector components are finite. */
  static bool finiteVector(const Vector3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }

  /** @brief Clamps to non-negative and substitutes fallback on non-finite input. */
  static double sanitizeNonNegative(double value, double fallback) {
    if (!std::isfinite(value)) {
      return fallback;
    }
    return std::max(0.0, value);
  }

  static Config sanitizeConfig(const Config& config) {
    Config sanitized = config;
    if (!finiteVector(sanitized.gravity_mps2)) {
      sanitized.gravity_mps2 = Vector3(0.0, 0.0, -9.81);
    }
    sanitized.effective_gravity_scale = std::clamp(
        sanitizeNonNegative(sanitized.effective_gravity_scale, 1.0), 0.0,
        5.0);
    sanitized.air_density_kgpm3 = std::clamp(
        sanitizeNonNegative(sanitized.air_density_kgpm3, 1.225), 0.0, 5.0);
    sanitized.drag_scale =
        std::clamp(sanitizeNonNegative(sanitized.drag_scale, 1.0), 0.0, 10.0);
    sanitized.ground_height_m = std::isfinite(sanitized.ground_height_m)
                                    ? sanitized.ground_height_m
                                    : 0.0;
    sanitized.rolling_friction_per_s =
        sanitizeNonNegative(sanitized.rolling_friction_per_s, 1.2);
    sanitized.min_bounce_speed_mps =
        sanitizeNonNegative(sanitized.min_bounce_speed_mps, 0.1);
    sanitized.max_substep_s = std::clamp(
        sanitizeNonNegative(sanitized.max_substep_s, 0.01), 1e-4, 0.05);
    return sanitized;
  }

  static BoxProperties sanitizeBoxProperties(const BoxProperties& props) {
    BoxProperties sanitized = props;
    sanitized.mass_kg =
        std::max(1e-6, sanitizeNonNegative(sanitized.mass_kg, 0.27));
    sanitized.dimensions_m.x =
        std::max(1e-4, sanitizeNonNegative(sanitized.dimensions_m.x, 0.24));
    sanitized.dimensions_m.y =
        std::max(1e-4, sanitizeNonNegative(sanitized.dimensions_m.y, 0.24));
    sanitized.dimensions_m.z =
        std::max(1e-4, sanitizeNonNegative(sanitized.dimensions_m.z, 0.24));
    sanitized.drag_coefficient = std::clamp(
        sanitizeNonNegative(sanitized.drag_coefficient, 0.47), 0.0, 5.0);
    sanitized.reference_area_m2 = sanitizeNonNegative(
        sanitized.reference_area_m2, defaultReferenceAreaM2(sanitized));
    if (sanitized.reference_area_m2 <= 0.0) {
      sanitized.reference_area_m2 = defaultReferenceAreaM2(sanitized);
    }
    sanitized.restitution = std::clamp(
        std::isfinite(sanitized.restitution) ? sanitized.restitution : 0.45,
        0.0, 1.0);
    return sanitized;
  }

  static double defaultReferenceAreaM2(const BoxProperties& props) {
    const Vector3 dims = props.dimensions_m;
    return std::max(dims.x * dims.y,
                    std::max(dims.x * dims.z, dims.y * dims.z));
  }

  /** @brief Replaces non-finite state components with zero vectors. */
  static void sanitizeState(BoxState& state) {
    if (!finiteVector(state.position_m)) {
      state.position_m = Vector3::zero();
    }
    if (!finiteVector(state.velocity_mps)) {
      state.velocity_mps = Vector3::zero();
    }
  }

  /**
   * @brief Computes linear acceleration from gravity and drag terms.
   * @param velocity_mps Current linear velocity.
   * @return World-frame acceleration in meters per second squared.
   */
  Vector3 computeAcceleration(const Vector3& velocity_mps) const {
    const Vector3 gravity_mps2 =
        config_.gravity_mps2 * config_.effective_gravity_scale;
    const double drag_area_m2 = dragReferenceAreaM2(velocity_mps);
    const Vector3 drag_force_n =
        Vector3::dragForce(velocity_mps, box_properties_.drag_coefficient,
                           drag_area_m2, config_.air_density_kgpm3) *
        config_.drag_scale;
    const Vector3 accel_mps2 = gravity_mps2 +
                               drag_force_n *
                                   (1.0 / std::max(1e-9, box_properties_.mass_kg));
    return finiteVector(accel_mps2) ? accel_mps2 : gravity_mps2;
  }

  /**
   * @brief Returns the effective drag reference area for the current velocity.
   * @param velocity_world World-space velocity used to infer projected area.
   * @return Effective reference area in square meters.
   */
  double dragReferenceAreaM2(const Vector3& velocity_world) const {
    if (box_properties_.reference_area_m2 > 0.0) {
      return box_properties_.reference_area_m2;
    }

    const Vector3 dims = box_properties_.dimensions_m;
    if (dims.x <= 0.0 || dims.y <= 0.0 || dims.z <= 0.0) {
      return 0.0;
    }

    const Vector3 direction = velocity_world.isZero() ? Vector3::unitX()
                                                      : velocity_world.normalized();
    return std::abs(direction.x) * dims.y * dims.z +
           std::abs(direction.y) * dims.x * dims.z +
           std::abs(direction.z) * dims.x * dims.y;
  }

  /**
   * @brief Resolves contact against the ground plane with bounce and rolling
   * friction.
   * @param dt_s Substep duration used for planar friction decay.
   */
  void resolveGroundContact(double dt_s) {
    const double floor_z =
        config_.ground_height_m + 0.5 * box_properties_.dimensions_m.z;
    if (!std::isfinite(floor_z) || state_.position_m.z > floor_z) {
      return;
    }

    state_.position_m.z = floor_z;
    if (state_.velocity_mps.z < -config_.min_bounce_speed_mps) {
      state_.velocity_mps.z =
          -state_.velocity_mps.z *
          std::clamp(box_properties_.restitution, 0.0, 1.0);
    } else {
      state_.velocity_mps.z = 0.0;
    }

    const double planar_speed = state_.velocity_mps.planarSpeed();
    if (planar_speed <= 1e-9) {
      state_.velocity_mps.x = 0.0;
      state_.velocity_mps.y = 0.0;
      return;
    }

    const double decay =
        std::max(0.0, 1.0 - config_.rolling_friction_per_s * dt_s);
    state_.velocity_mps.x *= decay;
    state_.velocity_mps.y *= decay;
  }

  Config config_{};
  BoxProperties box_properties_{};
  BoxState state_{};
};

}  // namespace frcsim