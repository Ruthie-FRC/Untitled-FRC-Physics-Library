#include <cassert>

#include "frcsim/arena/evergreen_arena.hpp"
#include "frcsim/arena/rebuilt2026_arena.hpp"

int main() {
    frcsim::EvergreenArena evergreen;
    frcsim::Rebuilt2026Arena rebuilt;

    // Field maps should have loaded obstacles and goals for both presets.
    assert(!evergreen.gamepieceSim().fieldElements().empty());
    assert(!evergreen.gamepieceSim().goals().empty());

    assert(!rebuilt.gamepieceSim().fieldElements().empty());
    assert(!rebuilt.gamepieceSim().goals().empty());

    // Minimal runtime sanity for each arena as vendordep-exposed API.
    frcsim::BallGamepieceSim::RobotState robot;
    robot.position_m = frcsim::Vector3(1.0, 1.0, 0.0);

    const auto e_robot = evergreen.addRobot(robot);
    const auto r_robot = rebuilt.addRobot(robot);

    frcsim::BallGamepieceSim::ExitTrajectoryParameters cmd;
    cmd.pitch_rad = 0.45;
    cmd.mechanism_speed_mps = 7.0;
    cmd.gamepiece_type = frcsim::BallGamepieceSim::GamePieceType::kBall;

    evergreen.gamepieceSim().fireProjectile(e_robot, cmd, true);
    rebuilt.gamepieceSim().fireProjectile(r_robot, cmd, true);

    for (int i = 0; i < 120; ++i) {
        evergreen.simulationPeriodic();
        rebuilt.simulationPeriodic();
    }

    assert(evergreen.gamepieceSim().countProjectiles() == 0);
    assert(rebuilt.gamepieceSim().countProjectiles() == 0);

    return 0;
}
