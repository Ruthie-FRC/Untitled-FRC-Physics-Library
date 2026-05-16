// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

#pragma once

#include "frcsim/gamepiece/box_physics.hpp"

namespace frcsim::BoxGamepiecePresets {

/**
 * @brief Baseline evergreen box properties for open-ended sandbox use.
 * @return Box physical constants tuned for a square hitbox default.
 */
inline BoxPhysicsSim3D::BoxProperties evergreenBoxProperties() {
  BoxPhysicsSim3D::BoxProperties properties{};
  properties.mass_kg = 0.24;
  properties.dimensions_m = Vector3(0.24, 0.24, 0.24);
  properties.drag_coefficient = 0.50;
  properties.reference_area_m2 = 0.0;
  properties.restitution = 0.48;
  return properties;
}

/**
 * @brief Baseline evergreen box physics config for open-ended sandbox use.
 * @return BoxPhysicsSim3D::Config with default environment values.
 */
inline BoxPhysicsSim3D::Config evergreenBoxConfig() {
  BoxPhysicsSim3D::Config config{};
  config.gravity_mps2 = Vector3(0.0, 0.0, -9.81);
  config.air_density_kgpm3 = 1.225;
  config.ground_height_m = 0.0;
  config.rolling_friction_per_s = 1.5;
  config.min_bounce_speed_mps = 0.06;
  return config;
}

/**
 * @brief Example 2026 box properties for a rectangular-prism gamepiece.
 * @return Box physical constants for a non-cubic hitbox example.
 */
inline BoxPhysicsSim3D::BoxProperties season2026BoxProperties() {
  BoxPhysicsSim3D::BoxProperties properties{};
  properties.mass_kg = 0.30;
  properties.dimensions_m = Vector3(0.30, 0.30, 0.20);
  properties.drag_coefficient = 0.58;
  properties.reference_area_m2 = 0.0;
  properties.restitution = 0.52;
  return properties;
}

/**
 * @brief Example 2026 box config derived from the evergreen baseline.
 * @return Box physics environment tuned for box hitbox gameplay.
 */
inline BoxPhysicsSim3D::Config season2026BoxConfig() {
  BoxPhysicsSim3D::Config config = evergreenBoxConfig();
  config.rolling_friction_per_s = 1.8;
  return config;
}

}  // namespace frcsim::BoxGamepiecePresets