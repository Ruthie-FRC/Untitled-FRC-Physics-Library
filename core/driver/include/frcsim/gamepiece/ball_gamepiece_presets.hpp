#pragma once

#include "frcsim/gamepiece/ball_gamepiece_sim.hpp"

namespace frcsim::BallGamepiecePresets {

/**
 * @brief Baseline evergreen ball properties for open-ended sandbox use.
 * @return Ball physical constants tuned for evergreen defaults.
 */
inline BallPhysicsSim3D::BallProperties evergreenBallProperties() {
    BallPhysicsSim3D::BallProperties properties{};
    properties.mass_kg = 0.24;
    properties.radius_m = 0.09;
    properties.drag_coefficient = 0.50;
    properties.reference_area_m2 = 3.14159265358979323846 * properties.radius_m * properties.radius_m;
    properties.restitution = 0.48;
    return properties;
}

/**
 * @brief Baseline evergreen ball physics config for open-ended sandbox use.
 * @return BallPhysicsSim3D::Config with default environment/drag values.
 */
inline BallPhysicsSim3D::Config evergreenBallConfig() {
    BallPhysicsSim3D::Config config{};
    config.gravity_mps2 = Vector3(0.0, 0.0, -9.81);
    config.air_density_kgpm3 = 1.225;
    config.magnus_coefficient = 1.0e-4;
    config.ground_height_m = 0.0;
    config.rolling_friction_per_s = 1.5;
    config.min_bounce_speed_mps = 0.06;
    return config;
}

/**
 * @brief Baseline evergreen field bounds and wall-contact coefficients.
 * @return Field configuration for the evergreen benchmark arena.
 */
inline BallGamepieceSim::FieldConfig evergreenFieldConfig() {
    BallGamepieceSim::FieldConfig config{};
    config.min_corner_m = Vector3(0.0, 0.0, 0.0);
    config.max_corner_m = Vector3(16.54, 8.21, 3.0);
    config.wall_restitution = 0.25;
    config.wall_friction = 0.2;
    return config;
}

/**
 * @brief Season 2026 field config with net boundary tuning enabled.
 * @return Field configuration specialized for the 2026 season preset.
 */
inline BallGamepieceSim::FieldConfig season2026FieldConfig() {
    BallGamepieceSim::FieldConfig config = evergreenFieldConfig();
    config.net_boundary_user_id = 2026;
    config.wall_restitution = 0.22;
    config.wall_friction = 0.25;
    return config;
}

/**
 * @brief Season 2026 game ball properties: 15 cm diameter, 0.216 kg.
 * @return Ball physical constants for the 2026 gamepiece.
 */
inline BallPhysicsSim3D::BallProperties season2026BallProperties() {
    BallPhysicsSim3D::BallProperties properties{};
    properties.mass_kg = 0.216;
    properties.radius_m = 0.15 * 0.5;
    properties.drag_coefficient = 0.58;
    properties.reference_area_m2 = 3.14159265358979323846 * properties.radius_m * properties.radius_m;
    properties.restitution = 0.52;
    return properties;
}

/**
 * @brief Season 2026 ball config derived from evergreen baseline.
 * @return Ball physics environment tuned for 2026 gameplay.
 */
inline BallPhysicsSim3D::Config season2026BallConfig() {
    BallPhysicsSim3D::Config config = evergreenBallConfig();
    config.magnus_coefficient = 1.2e-4;
    config.rolling_friction_per_s = 1.8;
    return config;
}

/**
 * @brief Applies the built-in Season 2026 net boundary and goal zone to a simulator instance.
 * @param sim Simulator instance to mutate.
 */
inline void configureSeason2026Field(BallGamepieceSim& sim) {
    sim.setFieldConfig(season2026FieldConfig());
    sim.fieldElements().clear();
    sim.goals().clear();

    EnvironmentalBoundary net{};
    net.type = BoundaryType::kBox;
    net.user_id = 2026;
    net.position_m = Vector3(7.0, 4.1, 1.7);
    net.half_extents_m = Vector3(0.4, 0.4, 0.5);
    net.is_active = true;
    sim.addFieldElement(net);

    BallGamepieceSim::GoalZone hub_goal{};
    hub_goal.shape = BallGamepieceSim::GoalZone::Shape::kBox;
    hub_goal.center_m = net.position_m;
    hub_goal.half_extents_m = Vector3(0.35, 0.35, 0.45);
    hub_goal.accepted_type = "Ball";
    hub_goal.require_positive_vertical_velocity = false;
    sim.addGoalZone(hub_goal);
}

}  // namespace frcsim::BallGamepiecePresets
