#include <cassert>
#include <cmath>

#include "frcsim/gamepiece/ball_gamepiece_sim.hpp"
#include "frcsim/gamepiece/ball_gamepiece_presets.hpp"

int main() {
    frcsim::BallGamepieceSim::FieldConfig field;
    field.net_boundary_user_id = 2026;

    frcsim::BallGamepieceSim sim(field);
    sim.setSimulationSubsteps(5);

    frcsim::BallGamepieceSim::RobotState robot_a;
    robot_a.position_m = frcsim::Vector3(1.0, 2.0, 0.0);
    robot_a.velocity_mps = frcsim::Vector3(3.0, 0.0, 0.0);
    robot_a.yaw_rad = 0.0;
    robot_a.intake_enabled = true;
    robot_a.intake_radius_m = 0.35;

    frcsim::BallGamepieceSim::RobotState robot_b;
    robot_b.position_m = frcsim::Vector3(1.7, 2.0, 0.0);
    robot_b.velocity_mps = frcsim::Vector3(-1.0, 0.0, 0.0);
    robot_b.yaw_rad = 0.0;

    const std::size_t robot_a_id = sim.addRobot(robot_a);
    const std::size_t robot_b_id = sim.addRobot(robot_b);

    const auto default_props = frcsim::BallGamepiecePresets::season2026BallProperties();
    assert(std::fabs(default_props.mass_kg - 0.216) < 1e-9);
    assert(std::fabs(default_props.radius_m - 0.075) < 1e-9);

    for (int i = 0; i < 8; ++i) {
        frcsim::BallPhysicsSim3D::BallState state;
        state.position_m = frcsim::Vector3(1.35 + 0.08 * i, 2.0 + ((i % 2 == 0) ? 0.03 : -0.03), 0.075);
        sim.addBall(state, frcsim::BallGamepiecePresets::season2026BallConfig(),
                frcsim::BallGamepiecePresets::season2026BallProperties());
    }

    assert(sim.countBalls() == 8);

    frcsim::EnvironmentalBoundary net;
    net.type = frcsim::BoundaryType::kBox;
    net.position_m = frcsim::Vector3(7.0, 2.0, 1.6);
    net.half_extents_m = frcsim::Vector3(0.35, 0.35, 0.45);
    net.user_id = 2026;
    net.is_active = true;
    sim.addFieldElement(net);

    frcsim::EnvironmentalBoundary wall;
    wall.type = frcsim::BoundaryType::kPlane;
    wall.position_m = frcsim::Vector3(4.0, 0.0, 0.0);
    wall.orientation = frcsim::Quaternion::fromAxisAngle(frcsim::Vector3::unitY(), -1.57079632679);
    wall.restitution = 0.5;
    wall.friction_coefficient = 0.15;
    wall.is_active = true;
    sim.addFieldElement(wall);

    for (int i = 0; i < 15; ++i) {
        sim.step(0.02);
    }

    assert(sim.robots()[robot_a_id].carried_ball_index != frcsim::BallGamepieceSim::kNoBall);

    const double relative_speed_before = (robot_a.velocity_mps - robot_b.velocity_mps).norm();
    const double relative_speed_after =
        (sim.robots()[robot_a_id].velocity_mps - sim.robots()[robot_b_id].velocity_mps).norm();
    assert(relative_speed_after < relative_speed_before + 1e-6);

    frcsim::BallGamepieceSim::ExitTrajectoryParameters fire;
    fire.launch_offset_m = frcsim::Vector3(0.45, 0.0, 0.55);
    fire.yaw_offset_rad = 0.0;
    fire.pitch_rad = 0.85;
    fire.mechanism_speed_mps = 12.0;
    fire.estimated_exit_velocity_mps = 16.5;
    fire.spin_radps = frcsim::Vector3(0.0, 40.0, 0.0);

    const bool fired = sim.fireBall(robot_a_id, fire);
    assert(fired);

    for (int i = 0; i < 240; ++i) {
        sim.step(0.02);
    }

    assert(sim.countBalls() == 8);
    assert(sim.countScoredBalls() <= sim.countBalls());

    // Maple-style projectile lifecycle: in-flight projectile can hit goal or become grounded piece.
    frcsim::BallGamepieceSim projectile_sim;
    projectile_sim.setFieldConfig(frcsim::BallGamepiecePresets::evergreenFieldConfig());

    frcsim::BallGamepieceSim::GamePieceInfo projectile_ball_type;
    projectile_ball_type.type = frcsim::BallGamepieceSim::GamePieceType::kBall;
    projectile_ball_type.physics_config = frcsim::BallGamepiecePresets::season2026BallConfig();
    projectile_ball_type.ball_properties = frcsim::BallGamepiecePresets::season2026BallProperties();
    projectile_ball_type.spawn_on_ground_after_projectile = true;
    projectile_sim.registerGamePieceType(projectile_ball_type);

    frcsim::BallGamepieceSim::RobotState launcher;
    launcher.position_m = frcsim::Vector3(1.0, 1.0, 0.0);
    launcher.yaw_rad = 0.0;
    const std::size_t launcher_id = projectile_sim.addRobot(launcher);

    frcsim::BallGamepieceSim::ExitTrajectoryParameters projectile_fire;
    projectile_fire.launch_offset_m = frcsim::Vector3(0.3, 0.0, 0.8);
    projectile_fire.pitch_rad = 0.35;
    projectile_fire.mechanism_speed_mps = 2.5;
    projectile_fire.gamepiece_type = frcsim::BallGamepieceSim::GamePieceType::kBall;

    projectile_sim.fireProjectile(launcher_id, projectile_fire, true);
    for (int i = 0; i < 200; ++i) {
        projectile_sim.step(0.01);
    }

    // Maple-style conversion: projectile becomes grounded piece after touch-ground.
    assert(projectile_sim.countProjectiles() == 0);
    assert(projectile_sim.countBalls() > 0);

    sim.robots()[robot_a_id].position_m = frcsim::Vector3(16.50, 2.0, 0.0);
    sim.robots()[robot_a_id].velocity_mps = frcsim::Vector3(2.0, 0.0, 0.0);
    sim.step(0.02);
    assert(sim.robots()[robot_a_id].velocity_mps.x == 0.0);

    bool some_ball_moved = false;
    for (const auto& ball : sim.balls()) {
        if (ball.sim.state().position_m.x > 2.2) {
            some_ball_moved = true;
            break;
        }
    }
    assert(some_ball_moved);

    // Ball-ball collision regression: two free balls moving head-on should
    // exchange momentum direction after contact.
    frcsim::BallGamepieceSim collision_sim;
    collision_sim.setSimulationSubsteps(6);
    frcsim::BallPhysicsSim3D::Config collision_cfg =
        frcsim::BallGamepiecePresets::season2026BallConfig();
    collision_cfg.drag_scale = 0.0;
    collision_cfg.magnus_scale = 0.0;
    collision_cfg.rolling_friction_per_s = 0.0;

    const auto collision_props =
        frcsim::BallGamepiecePresets::season2026BallProperties();

    frcsim::BallPhysicsSim3D::BallState left;
    left.position_m = frcsim::Vector3(4.0, 3.0, collision_props.radius_m);
    left.velocity_mps = frcsim::Vector3(1.5, 0.0, 0.0);
    collision_sim.addBall(left, collision_cfg, collision_props);

    frcsim::BallPhysicsSim3D::BallState right;
    right.position_m =
        frcsim::Vector3(4.0 + 2.0 * collision_props.radius_m + 0.02, 3.0,
                        collision_props.radius_m);
    right.velocity_mps = frcsim::Vector3(-1.5, 0.0, 0.0);
    collision_sim.addBall(right, collision_cfg, collision_props);

    for (int i = 0; i < 20; ++i) {
      collision_sim.step(0.01);
    }

    const auto& left_after = collision_sim.balls()[0].sim.state();
    const auto& right_after = collision_sim.balls()[1].sim.state();
    assert(left_after.velocity_mps.x < 0.0);
    assert(right_after.velocity_mps.x > 0.0);

        // Velocity-dependent restitution regression: high-speed impacts should be
        // less elastic than low-speed impacts.
        auto run_head_on_restitution_ratio =
                [&](double speed_mps) -> double {
            frcsim::BallGamepieceSim::FieldConfig cfg;
            cfg.ball_ball_contact_restitution = 0.8;
            cfg.ball_ball_contact_friction = 0.0;
            cfg.ball_ball_restitution_reference_speed_mps = 2.0;
            cfg.ball_ball_restitution_speed_exponent = 0.25;
            cfg.ball_ball_restitution_min_scale = 0.4;

            frcsim::BallGamepieceSim speed_sim(cfg);
            speed_sim.setSimulationSubsteps(8);

            frcsim::BallPhysicsSim3D::Config speed_cfg =
                    frcsim::BallGamepiecePresets::season2026BallConfig();
            speed_cfg.drag_scale = 0.0;
            speed_cfg.magnus_scale = 0.0;
            speed_cfg.rolling_friction_per_s = 0.0;

            const auto props = frcsim::BallGamepiecePresets::season2026BallProperties();

            frcsim::BallPhysicsSim3D::BallState a;
            a.position_m = frcsim::Vector3(5.0, 4.0, props.radius_m);
            a.velocity_mps = frcsim::Vector3(speed_mps, 0.0, 0.0);
            speed_sim.addBall(a, speed_cfg, props);

            frcsim::BallPhysicsSim3D::BallState b;
            b.position_m =
                    frcsim::Vector3(5.0 + 2.0 * props.radius_m + 0.03, 4.0, props.radius_m);
            b.velocity_mps = frcsim::Vector3(-speed_mps, 0.0, 0.0);
            speed_sim.addBall(b, speed_cfg, props);

            const double relative_in = 2.0 * speed_mps;
            for (int i = 0; i < 30; ++i) {
                speed_sim.step(0.005);
            }

            const double relative_out =
                    std::abs(speed_sim.balls()[1].sim.state().velocity_mps.x -
                                     speed_sim.balls()[0].sim.state().velocity_mps.x);
            return relative_out / relative_in;
        };

        const double low_speed_ratio = run_head_on_restitution_ratio(1.0);
        const double high_speed_ratio = run_head_on_restitution_ratio(5.0);
        assert(high_speed_ratio < low_speed_ratio - 0.05);

        // Wall velocity-dependent restitution regression: high-speed impacts should
        // lose more energy than low-speed impacts.
        auto run_wall_restitution_ratio = [&](double speed_mps) -> double {
            frcsim::BallGamepieceSim::FieldConfig cfg;
            cfg.wall_restitution = 0.85;
            cfg.wall_restitution_reference_speed_mps = 2.0;
            cfg.wall_restitution_speed_exponent = 0.25;
            cfg.wall_restitution_min_scale = 0.4;
            cfg.wall_friction = 0.0;

            frcsim::BallGamepieceSim wall_sim(cfg);
            wall_sim.setSimulationSubsteps(8);

            frcsim::BallPhysicsSim3D::Config ball_cfg =
                    frcsim::BallGamepiecePresets::season2026BallConfig();
            ball_cfg.drag_scale = 0.0;
            ball_cfg.magnus_scale = 0.0;
            ball_cfg.rolling_friction_per_s = 0.0;

            const auto props = frcsim::BallGamepiecePresets::season2026BallProperties();
            frcsim::BallPhysicsSim3D::BallState state;
            state.position_m = frcsim::Vector3(props.radius_m + 0.20, 3.0, props.radius_m);
            state.velocity_mps = frcsim::Vector3(-speed_mps, 0.0, 0.0);
            wall_sim.addBall(state, ball_cfg, props);

            for (int i = 0; i < 40; ++i) {
                wall_sim.step(0.005);
            }

            const double post_speed = wall_sim.balls()[0].sim.state().velocity_mps.x;
            return std::max(0.0, post_speed) / speed_mps;
        };

        const double wall_low_ratio = run_wall_restitution_ratio(1.0);
        const double wall_high_ratio = run_wall_restitution_ratio(5.0);
        assert(wall_high_ratio < wall_low_ratio - 0.05);

        // Ball-ball spin transfer regression: glancing collisions should induce
        // spin from tangential friction impulse.
        frcsim::BallGamepieceSim::FieldConfig spin_field;
        spin_field.ball_ball_contact_restitution = 0.4;
        spin_field.ball_ball_contact_friction = 0.8;
        spin_field.ball_ball_spin_transfer_gain = 0.35;
        spin_field.free_ball_spin_decay_per_s = 0.0;
        frcsim::BallGamepieceSim spin_sim(spin_field);
        spin_sim.setSimulationSubsteps(8);

        frcsim::BallPhysicsSim3D::Config spin_cfg =
                frcsim::BallGamepiecePresets::season2026BallConfig();
        spin_cfg.drag_scale = 0.0;
        spin_cfg.magnus_scale = 0.0;
        spin_cfg.rolling_friction_per_s = 0.0;
        const auto spin_props = frcsim::BallGamepiecePresets::season2026BallProperties();

        frcsim::BallPhysicsSim3D::BallState spin_a;
        spin_a.position_m = frcsim::Vector3(6.0, 4.0, spin_props.radius_m);
        spin_a.velocity_mps = frcsim::Vector3(2.5, 0.0, 0.0);
        spin_sim.addBall(spin_a, spin_cfg, spin_props);

        frcsim::BallPhysicsSim3D::BallState spin_b;
        spin_b.position_m = frcsim::Vector3(
                6.0 + 2.0 * spin_props.radius_m + 0.03, 4.0 + 0.08,
                spin_props.radius_m);
        spin_b.velocity_mps = frcsim::Vector3(0.0, 0.0, 0.0);
        spin_sim.addBall(spin_b, spin_cfg, spin_props);

        for (int i = 0; i < 50; ++i) {
            spin_sim.step(0.005);
        }

        const double spin_norm_a = spin_sim.balls()[0].sim.state().spin_radps.norm();
        const double spin_norm_b = spin_sim.balls()[1].sim.state().spin_radps.norm();
        assert(spin_norm_a > 1e-3 || spin_norm_b > 1e-3);

        // CCD regression: fast ball should not tunnel through a thin box boundary.
        frcsim::BallGamepieceSim::FieldConfig ccd_field;
        ccd_field.ccd_enabled = true;
        ccd_field.ccd_speed_threshold_mps = 1.0;
        ccd_field.wall_friction = 0.0;
        frcsim::BallGamepieceSim ccd_sim(ccd_field);
        ccd_sim.setSimulationSubsteps(1);

        frcsim::EnvironmentalBoundary thin_box;
        thin_box.type = frcsim::BoundaryType::kBox;
        thin_box.position_m = frcsim::Vector3(2.0, 2.0, 0.2);
        thin_box.half_extents_m = frcsim::Vector3(0.01, 0.8, 0.2);
        thin_box.restitution = 0.8;
        thin_box.friction_coefficient = 0.0;
        thin_box.is_active = true;
        ccd_sim.addFieldElement(thin_box);

        frcsim::BallPhysicsSim3D::Config ccd_cfg =
                frcsim::BallGamepiecePresets::season2026BallConfig();
        ccd_cfg.drag_scale = 0.0;
        ccd_cfg.magnus_scale = 0.0;
        ccd_cfg.rolling_friction_per_s = 0.0;
        const auto ccd_props = frcsim::BallGamepiecePresets::season2026BallProperties();

        frcsim::BallPhysicsSim3D::BallState ccd_ball;
        ccd_ball.position_m = frcsim::Vector3(1.0, 2.0, ccd_props.radius_m);
        ccd_ball.velocity_mps = frcsim::Vector3(45.0, 0.0, 0.0);
        ccd_sim.addBall(ccd_ball, ccd_cfg, ccd_props);
        ccd_sim.step(0.05);

        const auto ccd_after = ccd_sim.balls()[0].sim.state();
        assert(ccd_after.position_m.x < 2.0 + ccd_props.radius_m + 0.03);
        assert(ccd_after.velocity_mps.x < 0.0);

        // Sleeping/wake regression: resting balls should sleep and wake on impact.
        frcsim::BallGamepieceSim::FieldConfig sleep_field;
        sleep_field.sleeping_enabled = true;
        sleep_field.sleep_velocity_threshold_mps = 0.05;
        sleep_field.sleep_spin_threshold_radps = 0.2;
        sleep_field.sleep_frame_threshold = 4;
        sleep_field.ccd_enabled = false;
        frcsim::BallGamepieceSim sleep_sim(sleep_field);
        sleep_sim.setSimulationSubsteps(4);

        frcsim::BallPhysicsSim3D::Config sleep_cfg =
                frcsim::BallGamepiecePresets::season2026BallConfig();
        sleep_cfg.drag_scale = 0.0;
        sleep_cfg.magnus_scale = 0.0;
        const auto sleep_props = frcsim::BallGamepiecePresets::season2026BallProperties();

        frcsim::BallPhysicsSim3D::BallState sleeper;
        sleeper.position_m = frcsim::Vector3(7.0, 2.0, sleep_props.radius_m);
        sleeper.velocity_mps = frcsim::Vector3(0.0, 0.0, 0.0);
        sleep_sim.addBall(sleeper, sleep_cfg, sleep_props);

        for (int i = 0; i < 10; ++i) {
            sleep_sim.step(0.02);
        }
        assert(sleep_sim.balls()[0].sleeping);

        frcsim::BallPhysicsSim3D::BallState hitter;
        hitter.position_m =
                frcsim::Vector3(7.0 - 2.0 * sleep_props.radius_m - 0.05, 2.0,
                                                sleep_props.radius_m);
        hitter.velocity_mps = frcsim::Vector3(3.0, 0.0, 0.0);
        sleep_sim.addBall(hitter, sleep_cfg, sleep_props);

        for (int i = 0; i < 20; ++i) {
            sleep_sim.step(0.01);
        }
        assert(!sleep_sim.balls()[0].sleeping);

    // Sleeping integration regression: distant active field elements should not
    // wake an otherwise resting ball.
    frcsim::BallGamepieceSim::FieldConfig sleep_field_with_geometry;
    sleep_field_with_geometry.sleeping_enabled = true;
    sleep_field_with_geometry.sleep_velocity_threshold_mps = 0.05;
    sleep_field_with_geometry.sleep_spin_threshold_radps = 0.2;
    sleep_field_with_geometry.sleep_frame_threshold = 4;
    frcsim::BallGamepieceSim sleep_geo_sim(sleep_field_with_geometry);
    sleep_geo_sim.setSimulationSubsteps(4);

    frcsim::EnvironmentalBoundary far_geometry;
    far_geometry.type = frcsim::BoundaryType::kBox;
    far_geometry.position_m = frcsim::Vector3(14.0, 7.0, 0.4);
    far_geometry.half_extents_m = frcsim::Vector3(0.3, 0.3, 0.4);
    far_geometry.restitution = 0.7;
    far_geometry.friction_coefficient = 0.2;
    far_geometry.is_active = true;
    sleep_geo_sim.addFieldElement(far_geometry);

    frcsim::BallPhysicsSim3D::BallState sleep_geo_ball;
    sleep_geo_ball.position_m = frcsim::Vector3(2.0, 1.5, sleep_props.radius_m);
    sleep_geo_ball.velocity_mps = frcsim::Vector3(0.0, 0.0, 0.0);
    sleep_geo_sim.addBall(sleep_geo_ball, sleep_cfg, sleep_props);

    for (int i = 0; i < 12; ++i) {
      sleep_geo_sim.step(0.02);
    }
    assert(sleep_geo_sim.balls()[0].sleeping);

        // Full integration regression: run all math systems together and verify
        // finite, bounded, non-explosive behavior over a sustained horizon.
        frcsim::BallGamepieceSim::FieldConfig full_field;
        full_field.ccd_enabled = true;
        full_field.ccd_speed_threshold_mps = 6.0;
        full_field.sleeping_enabled = true;
        full_field.sleep_velocity_threshold_mps = 0.06;
        full_field.sleep_spin_threshold_radps = 0.5;
        full_field.sleep_frame_threshold = 8;
        full_field.solver_iterations = 5;
        full_field.baumgarte_beta = 0.25;
        full_field.baumgarte_slop_m = 0.004;
        full_field.ball_ball_contact_restitution = 0.55;
        full_field.ball_ball_contact_friction = 0.28;
        full_field.ball_ball_spin_transfer_gain = 0.22;
        full_field.free_ball_spin_decay_per_s = 0.3;
        full_field.net_boundary_user_id = 777;

        frcsim::BallGamepieceSim full_sim(full_field);
        full_sim.setSimulationSubsteps(6);

        frcsim::BallGamepieceSim::RobotState full_robot_a;
        full_robot_a.position_m = frcsim::Vector3(2.0, 2.0, 0.0);
        full_robot_a.velocity_mps = frcsim::Vector3(1.4, 0.2, 0.0);
        full_robot_a.yaw_rad = 0.2;
        full_robot_a.intake_enabled = true;
        full_robot_a.intake_radius_m = 0.32;
        full_sim.addRobot(full_robot_a);

        frcsim::BallGamepieceSim::RobotState full_robot_b;
        full_robot_b.position_m = frcsim::Vector3(12.0, 6.0, 0.0);
        full_robot_b.velocity_mps = frcsim::Vector3(-1.0, -0.4, 0.0);
        full_robot_b.yaw_rad = -0.6;
        full_sim.addRobot(full_robot_b);

        frcsim::EnvironmentalBoundary full_plane;
        full_plane.type = frcsim::BoundaryType::kPlane;
        full_plane.position_m = frcsim::Vector3(6.0, 0.0, 0.0);
        full_plane.orientation =
                frcsim::Quaternion::fromAxisAngle(frcsim::Vector3::unitY(), -1.57079632679);
        full_plane.restitution = 0.5;
        full_plane.friction_coefficient = 0.18;
        full_plane.is_active = true;
        full_sim.addFieldElement(full_plane);

        frcsim::EnvironmentalBoundary full_box;
        full_box.type = frcsim::BoundaryType::kBox;
        full_box.position_m = frcsim::Vector3(8.0, 4.0, 0.35);
        full_box.half_extents_m = frcsim::Vector3(0.08, 1.0, 0.35);
        full_box.restitution = 0.62;
        full_box.friction_coefficient = 0.22;
        full_box.is_active = true;
        full_sim.addFieldElement(full_box);

        frcsim::EnvironmentalBoundary full_cylinder;
        full_cylinder.type = frcsim::BoundaryType::kCylinder;
        full_cylinder.position_m = frcsim::Vector3(10.5, 3.8, 0.3);
        full_cylinder.radius_m = 0.22;
        full_cylinder.half_extents_m = frcsim::Vector3(0.0, 0.0, 0.5);
        full_cylinder.restitution = 0.55;
        full_cylinder.friction_coefficient = 0.2;
        full_cylinder.is_active = true;
        full_sim.addFieldElement(full_cylinder);

        frcsim::EnvironmentalBoundary full_net;
        full_net.type = frcsim::BoundaryType::kBox;
        full_net.position_m = frcsim::Vector3(15.0, 4.0, 1.7);
        full_net.half_extents_m = frcsim::Vector3(0.3, 0.5, 0.45);
        full_net.user_id = 777;
        full_net.is_active = true;
        full_sim.addFieldElement(full_net);

        frcsim::BallPhysicsSim3D::Config full_cfg =
                frcsim::BallGamepiecePresets::season2026BallConfig();
        full_cfg.max_substep_s = 0.004;
        full_cfg.drag_scale = 1.0;
        full_cfg.magnus_scale = 1.0;

        const auto full_props = frcsim::BallGamepiecePresets::season2026BallProperties();
        for (int i = 0; i < 18; ++i) {
            frcsim::BallPhysicsSim3D::BallState seed;
            seed.position_m = frcsim::Vector3(
                    1.2 + 0.7 * (i % 6),
                    1.1 + 0.8 * ((i / 6) % 3),
                    full_props.radius_m + 0.03 * (i % 2));
            seed.velocity_mps = frcsim::Vector3(
                    (i % 3 == 0 ? 6.5 : -3.8 + 0.22 * i),
                    (i % 2 == 0 ? 1.6 : -1.3),
                    (i % 5 == 0 ? 2.8 : 0.4));
            seed.spin_radps = frcsim::Vector3(0.0, (i % 2 == 0 ? 45.0 : -35.0), 6.0);
            full_sim.addBall(seed, full_cfg, full_props);
        }

        for (int step = 0; step < 900; ++step) {
            full_sim.step(0.005);
            const auto& balls = full_sim.balls();

            for (const auto& entity : balls) {
                const auto& s = entity.sim.state();
                assert(std::isfinite(s.position_m.x));
                assert(std::isfinite(s.position_m.y));
                assert(std::isfinite(s.position_m.z));
                assert(std::isfinite(s.velocity_mps.x));
                assert(std::isfinite(s.velocity_mps.y));
                assert(std::isfinite(s.velocity_mps.z));
                assert(std::isfinite(s.spin_radps.x));
                assert(std::isfinite(s.spin_radps.y));
                assert(std::isfinite(s.spin_radps.z));

                assert(std::fabs(s.velocity_mps.x) < 220.0);
                assert(std::fabs(s.velocity_mps.y) < 220.0);
                assert(std::fabs(s.velocity_mps.z) < 220.0);
                assert(s.position_m.x > -1.0 && s.position_m.x < 18.0);
                assert(s.position_m.y > -1.0 && s.position_m.y < 10.0);
                assert(s.position_m.z > -0.2 && s.position_m.z < 8.0);

                if (entity.sleeping) {
                    assert(s.velocity_mps.norm() <=
                                 full_field.sleep_velocity_threshold_mps + 0.03);
                }
            }

            for (std::size_t i = 0; i < balls.size(); ++i) {
                for (std::size_t j = i + 1; j < balls.size(); ++j) {
                    const auto& a = balls[i].sim.state();
                    const auto& b = balls[j].sim.state();
                    const double min_sep =
                            balls[i].sim.ballProperties().radius_m +
                            balls[j].sim.ballProperties().radius_m;
                    const double sep = (b.position_m - a.position_m).norm();
                    assert(sep + 0.03 >= min_sep);
                }
            }
        }

        assert(full_sim.countScoredBalls() <= full_sim.countBalls());

    return 0;
}
