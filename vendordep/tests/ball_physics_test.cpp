#include <cassert>
#include <cmath>
#include <limits>

#include "frcsim/gamepiece/ball_physics.hpp"

int main() {
    frcsim::BallPhysicsSim3D::Config config;
    config.gravity_mps2 = frcsim::Vector3(0.0, 0.0, -9.81);
    config.ground_height_m = 0.0;
    config.rolling_friction_per_s = 1.0;
    config.min_bounce_speed_mps = 0.05;

    frcsim::BallPhysicsSim3D::BallProperties properties;
    properties.mass_kg = 0.27;
    properties.radius_m = 0.09;
    properties.drag_coefficient = 0.47;
    properties.reference_area_m2 = 0.025;
    properties.restitution = 0.6;

    frcsim::BallPhysicsSim3D sim(config, properties);

    frcsim::BallPhysicsSim3D::BallState initial;
    initial.position_m = frcsim::Vector3(0.0, 0.0, properties.radius_m);
    sim.setState(initial);

    // Pickup when intake is close enough.
    sim.setCarrierPose(frcsim::Vector3(1.0, 0.0, 0.5), frcsim::Vector3(2.0, 0.0, 0.0));
    frcsim::BallPhysicsSim3D::PickupRequest request;
    request.intake_position_m = frcsim::Vector3(0.0, 0.0, properties.radius_m);
    request.capture_radius_m = 0.15;
    request.carry_offset_m = frcsim::Vector3(0.2, 0.0, 0.1);

    const bool picked_up = sim.requestPickup(request);
    assert(picked_up);
    assert(sim.state().held);

    // While held, ball should move with carrier and offset.
    sim.setCarrierPose(frcsim::Vector3(2.0, 0.0, 0.8), frcsim::Vector3(1.5, 0.0, 0.0));
    sim.step(0.02);
    assert(std::fabs(sim.state().position_m.x - 2.2) < 1e-9);
    assert(std::fabs(sim.state().position_m.z - 0.9) < 1e-9);

    // Shoot transitions to free-flight at muzzle state.
    const frcsim::Vector3 muzzle_position(2.2, 0.0, 1.0);
    const frcsim::Vector3 muzzle_velocity(8.0, 0.0, 5.0);
    const frcsim::Vector3 muzzle_spin(0.0, 35.0, 0.0);
    sim.shoot(muzzle_position, muzzle_velocity, muzzle_spin);
    assert(!sim.state().held);

    const frcsim::Vector3 start_position = sim.state().position_m;
    for (int i = 0; i < 30; ++i) {
        sim.step(0.01);
    }

    // Should have moved downrange and still be above floor after a short flight.
    assert(sim.state().position_m.x > start_position.x + 1.0);
    assert(sim.state().position_m.z > properties.radius_m);

    // Continue long enough for ground contact and bounce/roll behavior.
    for (int i = 0; i < 220; ++i) {
        sim.step(0.01);
    }

    assert(sim.state().position_m.z >= properties.radius_m - 1e-9);
    // Ground contact should eventually reduce vertical speed and keep it finite.
    assert(std::isfinite(sim.state().velocity_mps.z));

    // Drag should produce a shorter (non-parabolic) downrange trajectory than vacuum-like flight.
    frcsim::BallPhysicsSim3D::Config drag_cfg = config;
    drag_cfg.drag_scale = 1.0;
    drag_cfg.magnus_scale = 0.0;
    drag_cfg.effective_gravity_scale = 1.0;

    frcsim::BallPhysicsSim3D::Config no_drag_cfg = drag_cfg;
    no_drag_cfg.drag_scale = 0.0;

    frcsim::BallPhysicsSim3D with_drag(drag_cfg, properties);
    frcsim::BallPhysicsSim3D without_drag(no_drag_cfg, properties);

    frcsim::BallPhysicsSim3D::BallState ballistic_start;
    ballistic_start.position_m = frcsim::Vector3(0.0, 0.0, 1.0);
    with_drag.setState(ballistic_start);
    without_drag.setState(ballistic_start);
    with_drag.shoot(ballistic_start.position_m, frcsim::Vector3(10.0, 0.0, 6.0));
    without_drag.shoot(ballistic_start.position_m, frcsim::Vector3(10.0, 0.0, 6.0));

    for (int i = 0; i < 120; ++i) {
        with_drag.step(0.01);
        without_drag.step(0.01);
    }

    assert(without_drag.state().position_m.x > with_drag.state().position_m.x + 0.2);

    // Effective gravity scaling should affect arc drop.
    frcsim::BallPhysicsSim3D::Config reduced_g_cfg = no_drag_cfg;
    reduced_g_cfg.effective_gravity_scale = 0.5;
    frcsim::BallPhysicsSim3D reduced_g(reduced_g_cfg, properties);
    reduced_g.setState(ballistic_start);
    reduced_g.shoot(ballistic_start.position_m, frcsim::Vector3(10.0, 0.0, 6.0));

    frcsim::BallPhysicsSim3D nominal_g(no_drag_cfg, properties);
    nominal_g.setState(ballistic_start);
    nominal_g.shoot(ballistic_start.position_m, frcsim::Vector3(10.0, 0.0, 6.0));

    for (int i = 0; i < 100; ++i) {
        reduced_g.step(0.01);
        nominal_g.step(0.01);
    }

    assert(reduced_g.state().position_m.z > nominal_g.state().position_m.z + 0.2);

    // Invalid config/property/state values should fail closed to finite state updates.
    frcsim::BallPhysicsSim3D::Config bad_cfg;
    bad_cfg.gravity_mps2 = frcsim::Vector3(std::numeric_limits<double>::quiet_NaN(), 0.0, -9.81);
    bad_cfg.air_density_kgpm3 = -1.0;
    bad_cfg.drag_scale = std::numeric_limits<double>::infinity();
    bad_cfg.max_substep_s = 0.0;

    frcsim::BallPhysicsSim3D::BallProperties bad_props;
    bad_props.mass_kg = 0.0;
    bad_props.radius_m = -1.0;
    bad_props.reference_area_m2 = -0.5;
    bad_props.restitution = 2.0;

    frcsim::BallPhysicsSim3D hardened(bad_cfg, bad_props);
    frcsim::BallPhysicsSim3D::BallState bad_state;
    bad_state.position_m = frcsim::Vector3(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    bad_state.velocity_mps = frcsim::Vector3(std::numeric_limits<double>::infinity(), 0.0, 0.0);
    hardened.setState(bad_state);
    hardened.shoot(frcsim::Vector3(0.0, 0.0, 1.0),
                   frcsim::Vector3(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    hardened.step(0.02);

    const auto& hardened_state = hardened.state();
    assert(std::isfinite(hardened_state.position_m.x));
    assert(std::isfinite(hardened_state.position_m.y));
    assert(std::isfinite(hardened_state.position_m.z));
    assert(std::isfinite(hardened_state.velocity_mps.x));
    assert(std::isfinite(hardened_state.velocity_mps.y));
    assert(std::isfinite(hardened_state.velocity_mps.z));

    return 0;
}
