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

    return 0;
}
