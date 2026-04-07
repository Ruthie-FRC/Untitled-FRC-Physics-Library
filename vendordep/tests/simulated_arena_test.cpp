#include <cassert>

#include "frcsim/arena/simulated_arena.hpp"
#include "frcsim/gamepiece/ball_gamepiece_presets.hpp"

int main() {
    frcsim::SimulatedArena arena(frcsim::BallGamepiecePresets::season2026FieldConfig());

    frcsim::SimulatedArena::Timings timings;
    timings.robot_period_s = 0.02;
    timings.simulation_subticks_per_period = 5;
    arena.setTimings(timings);

    frcsim::SimulatedArena::FieldMap map;
    map.obstacles.push_back(
        frcsim::FieldObstacle::makeBox(frcsim::Vector3(3.0, 2.0, 0.5), frcsim::Vector3(0.2, 0.2, 0.5)));

    frcsim::GoalStructure goal;
    goal.shape = frcsim::GoalStructure::Shape::kSphere;
    goal.center_m = frcsim::Vector3(4.0, 2.0, 1.1);
    goal.radius_m = 0.8;
    goal.accepted_type = "Ball";
    map.goals.push_back(goal);

    arena.applyFieldMap(map);

    frcsim::BallGamepieceSim::RobotState robot;
    robot.position_m = frcsim::Vector3(1.0, 2.0, 0.0);
    robot.intake_enabled = false;
    robot.intake_radius_m = 0.4;
    robot.intake_capacity = 2;
    const std::size_t robot_id = arena.addRobot(robot);

    frcsim::BallPhysicsSim3D::BallState loose_ball;
    loose_ball.position_m = frcsim::Vector3(1.1, 2.0, 0.08);
    arena.gamepieceSim().addBall(loose_ball,
                                 frcsim::BallGamepiecePresets::season2026BallConfig(),
                                 frcsim::BallGamepiecePresets::season2026BallProperties());

    frcsim::IntakeSimulation::Config intake_cfg;
    intake_cfg.robot_index = robot_id;
    intake_cfg.targeted_type = "Ball";
    intake_cfg.capacity = 2;
    auto& intake = arena.addIntakeSimulation(intake_cfg);
    intake.setRunning(true);

    intake.update(arena.gamepieceSim());
    assert(intake.gamePiecesInIntakeCount() >= 1);
    assert(arena.gamepieceSim().countBalls() == 0);

    int callback_ticks = 0;
    arena.addCustomSimulation([&callback_ticks](int, frcsim::SimulatedArena&) { ++callback_ticks; });

    arena.simulationPeriodic();
    assert(callback_ticks == timings.simulation_subticks_per_period);

    frcsim::BallGamepieceSim::ExitTrajectoryParameters shot;
    shot.launch_offset_m = frcsim::Vector3(0.45, 0.0, 0.7);
    shot.pitch_rad = 0.55;
    shot.mechanism_speed_mps = 8.5;
    shot.gamepiece_type = "Ball";

    bool scored_callback_called = false;
    arena.gamepieceSim().fireProjectile(robot_id, shot, true, [&scored_callback_called]() { scored_callback_called = true; });

    for (int i = 0; i < 220; ++i) {
        arena.simulationPeriodic();
    }

    assert(arena.gamepieceSim().countProjectiles() == 0);
    assert(scored_callback_called || arena.gamepieceSim().countBalls() > 0);

    return 0;
}
