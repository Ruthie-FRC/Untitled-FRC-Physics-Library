#include <cassert>

#include "frcsim/mechanisms/shooter_wheel.hpp"
#include "frcsim/mechanisms/turret_shooter_sim.hpp"

int main() {
    frcsim::ShooterWheelSim::WheelConfig wheel;
    wheel.radius_m = 0.050;
    wheel.inertia_kgm2 = 0.0025;
    wheel.viscous_friction_nm_per_radps = 0.0007;
    wheel.ball_coupling = 0.9;

    const auto high_torque = frcsim::ShooterWheelSim::MotorConfig::krakenX60(1);
    const auto high_speed = frcsim::ShooterWheelSim::MotorConfig::falcon500(1);

    frcsim::ShooterWheelSim torque_sim(high_torque, wheel);
    frcsim::ShooterWheelSim speed_sim(high_speed, wheel);

    frcsim::ShooterWheelSim::ControlInput cmd;
    cmd.velocity_closed_loop = true;
    cmd.target_speed_radps = 400.0;
    cmd.velocity_kp = 0.06;
    cmd.current_limit_a = 90.0;
    cmd.friction_voltage_v = 0.2;

    for (int i = 0; i < 250; ++i) {
        torque_sim.step(0.01, cmd);
        speed_sim.step(0.01, cmd);
    }

    assert(torque_sim.angularSpeedRadps() > 0.0);
    assert(speed_sim.angularSpeedRadps() > 0.0);
    assert(torque_sim.estimatedExitVelocityMps() > 0.0);
    assert(speed_sim.estimatedExitVelocityMps() > 0.0);

    // Different motor configs should produce distinguishable settled speeds under same control.
    assert(std::abs(torque_sim.angularSpeedRadps() - speed_sim.angularSpeedRadps()) > 1e-3);

    frcsim::TurretShooterSim turret;
    frcsim::BallPhysicsSim3D::BallState preload;
    preload.held = true;
    turret.ball().setState(preload);

    turret.shootWithWheel(speed_sim);
    assert(!turret.ball().state().held);
    assert(turret.ball().state().velocity_mps.norm() > 0.1);

    return 0;
}
