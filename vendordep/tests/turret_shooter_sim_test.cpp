#include <cassert>
#include <cmath>

#include "frcsim/mechanisms/turret_shooter_sim.hpp"

int main() {
    frcsim::TurretFlywheelSim::Config config;
    config.turret_mount_position_m = frcsim::Vector3(1.0, 2.0, 0.6);
    config.muzzle_offset_local_m = frcsim::Vector3(0.4, 0.0, 0.2);
    config.intake_offset_local_m = frcsim::Vector3(0.2, 0.0, 0.0);
    config.intake_capture_radius_m = 0.25;

    config.differential.m00 = 0.5;
    config.differential.m01 = 0.5;
    config.differential.m10 = 0.5;
    config.differential.m11 = -0.5;

    frcsim::TurretFlywheelSim sim(config);

    frcsim::DoubleDifferentialMechanism::MotorState motors;
    // Yaw=(a+b)/2=0.5 rad, Pitch=(a-b)/2=0.1 rad.
    motors.motor_a_position_rad = 0.6;
    motors.motor_b_position_rad = 0.4;
    sim.setMotorState(motors);
    sim.setBaseVelocity(frcsim::Vector3(1.0, 0.0, 0.0));

    frcsim::BallPhysicsSim3D::BallState loose_ball;
    loose_ball.position_m = sim.intakePositionWorld();
    sim.ball().setState(loose_ball);

    const bool captured = sim.pickupBallNearby();
    assert(captured);
    assert(sim.ball().state().held);

    // Held ball should follow mechanism while stepping.
    sim.step(0.02);
    assert((sim.ball().state().position_m - sim.intakePositionWorld()).norm() < 0.4);

    const frcsim::Vector3 start = sim.muzzlePositionWorld();
    sim.shoot(12.0, frcsim::Vector3(0.0, 50.0, 0.0));
    assert(!sim.ball().state().held);

    for (int i = 0; i < 40; ++i) {
        sim.step(0.01);
    }

    const frcsim::Vector3 end = sim.ball().state().position_m;
    assert(end.x > start.x + 2.0);
    assert(std::isfinite(end.z));

    return 0;
}
